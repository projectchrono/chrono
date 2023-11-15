// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================

#include <algorithm>
#include <climits>

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/ChCollisionInfo.h"

#include "chrono/collision/multicore/ChNarrowphase.h"
#include "chrono/collision/multicore/ChCollisionUtils.h"

#include "chrono/multicore_math/utility.h"

// Always include ChConfig.h *before* any Thrust headers!
#include "chrono/ChConfig.h"
#include <thrust/remove.h>
#include <thrust/sort.h>
#include <thrust/transform_reduce.h>
#include <thrust/count.h>
#include <thrust/iterator/constant_iterator.h>

namespace chrono {

using namespace chrono::ch_utils;

ChNarrowphase::ChNarrowphase()
    : algorithm(Algorithm::HYBRID),
      num_potential_rigid_contacts(0),
      num_potential_fluid_contacts(0),
      num_potential_rigid_fluid_contacts(0),
      cd_data(nullptr) {}

void ChNarrowphase::ClearContacts() {
    // Return now if no potential collisions.
    if (num_potential_rigid_contacts == 0) {
        cd_data->norm_rigid_rigid.resize(0);
        cd_data->cpta_rigid_rigid.resize(0);
        cd_data->cptb_rigid_rigid.resize(0);
        cd_data->dpth_rigid_rigid.resize(0);
        cd_data->erad_rigid_rigid.resize(0);
        cd_data->bids_rigid_rigid.resize(0);
    }
}

void ChNarrowphase::Process() {
    if (cd_data->state_data.num_fluid_bodies != 0) {
        ProcessFluid();
    }
    if (cd_data->num_rigid_shapes != 0) {
        ProcessRigids();
    } else {
        cd_data->c_counts_rigid_fluid.clear();
        cd_data->num_rigid_fluid_contacts = 0;
    }
}

void ChNarrowphase::ProcessRigids() {
    num_potential_rigid_contacts = cd_data->num_rigid_contacts;
    num_potential_rigid_fluid_contacts = cd_data->num_rigid_fluid_contacts;
    num_potential_fluid_contacts = cd_data->num_fluid_contacts;

    ClearContacts();

    // Transform Rigid body shapes to global coordinate system
    PreprocessLocalToParent();

    if (num_potential_rigid_contacts != 0) {
        ProcessRigidRigid();
    }

    if (cd_data->state_data.num_fluid_bodies != 0) {
        ProcessRigidFluid();
    }
}

// -----------------------------------------------------------------------------

int ChNarrowphase::PreprocessCount() {
    // Set the number of potential contact points for each collision pair
    contact_index.resize(num_potential_rigid_contacts + 1);

    if (algorithm == Algorithm::MPR) {
        // MPR always reports at most one contact per pair.
        Thrust_Fill(contact_index, 1);
    } else {
        // Analytical (and hence the hybrid) algorithms may produce different number
        // of contacts per pair, depending on the interacting shapes:
        //   - an interaction involving a sphere can produce at most one contact
        //   - an interaction involving a capsule can produce up to two contacts
        //   - a box-box interaction can produce up to 8 contacts

        // shape type (per shape)
        const shape_type* obj_data_T = cd_data->shape_data.typ_rigid.data();
        // encoded shape IDs (per collision pair)
        const long long* pair_shapeIDs = cd_data->pair_shapeIDs.data();

#pragma omp parallel for
        for (int index = 0; index < (signed)num_potential_rigid_contacts; index++) {
            // Identify the two candidate shapes and get their types.
            vec2 pair = I2(int(pair_shapeIDs[index] >> 32), int(pair_shapeIDs[index] & 0xffffffff));
            shape_type type1 = obj_data_T[pair.x];
            shape_type type2 = obj_data_T[pair.y];

            // Set the maximum number of possible contacts for this particular pair
            if (type1 == ChCollisionShape::Type::SPHERE || type2 == ChCollisionShape::Type::SPHERE) {
                contact_index[index] = 1;
            } else if (type1 == ChCollisionShape::Type::CAPSULE || type2 == ChCollisionShape::Type::CAPSULE) {
                contact_index[index] = 2;
            } else if (type1 == ChCollisionShape::Type::CYLSHELL || type2 == ChCollisionShape::Type::CYLSHELL) {
                contact_index[index] = 8;
            } else if (type1 == ChCollisionShape::Type::BOX && type2 == ChCollisionShape::Type::BOX) {
                contact_index[index] = 8;
            } else if ((type1 == ChCollisionShape::Type::BOX && type2 == ChCollisionShape::Type::TRIANGLE) ||
                       (type1 == ChCollisionShape::Type::TRIANGLE && type2 == ChCollisionShape::Type::BOX)) {
                contact_index[index] = 6;
            } else {
                contact_index[index] = 1;
            }
        }
    }

    contact_index[num_potential_rigid_contacts] = 0;

    // Calculate total number of potential contacts
    int num_potentialContacts = thrust::reduce(THRUST_PAR contact_index.begin(), contact_index.end());

    // Expand vector of shape IDs into contact_shapeIDs:
    // Replicate pair_shapeIDs[i] contact_index[i] times, for each potential contact for the collision pair 'i'
    cd_data->contact_shapeIDs.resize(num_potentialContacts);
    Thrust_Expand(contact_index.begin(), contact_index.end() - 1, cd_data->pair_shapeIDs.begin(),
                  cd_data->contact_shapeIDs.begin());

    // Set start index for the potential contacts for each collision pair
    Thrust_Exclusive_Scan(contact_index);
    assert(num_potentialContacts == (int)contact_index.back());

    // Return total number of potential contacts
    return num_potentialContacts;
}

void ChNarrowphase::PreprocessLocalToParent() {
    uint num_shapes = cd_data->num_rigid_shapes;

    const std::vector<int>& obj_data_T = cd_data->shape_data.typ_rigid;
    const std::vector<real3>& obj_data_A = cd_data->shape_data.ObA_rigid;
    const std::vector<quaternion>& obj_data_R = cd_data->shape_data.ObR_rigid;
    const std::vector<uint>& obj_data_ID = cd_data->shape_data.id_rigid;

    const std::vector<real3>& body_pos = *cd_data->state_data.pos_rigid;
    const std::vector<quaternion>& body_rot = *cd_data->state_data.rot_rigid;

    cd_data->shape_data.obj_data_A_global.resize(num_shapes);

    cd_data->shape_data.obj_data_R_global.resize(num_shapes);
    cd_data->shape_data.triangle_global.resize(cd_data->shape_data.triangle_rigid.size());

#pragma omp parallel for
    for (int index = 0; index < (signed)num_shapes; index++) {
        shape_type T = obj_data_T[index];

        // Get the identifier for the object associated with this collision shape
        uint ID = obj_data_ID[index];
        if (ID == UINT_MAX)
            continue;

        real3 pos = body_pos[ID];       // Get the global object position
        quaternion rot = body_rot[ID];  // Get the global object rotation

        cd_data->shape_data.obj_data_A_global[index] = TransformLocalToParent(pos, rot, obj_data_A[index]);
        if (T == ChCollisionShape::Type::TRIANGLE) {
            int start = cd_data->shape_data.start_rigid[index];
            cd_data->shape_data.triangle_global[start + 0] =
                TransformLocalToParent(pos, rot, cd_data->shape_data.triangle_rigid[start + 0]);
            cd_data->shape_data.triangle_global[start + 1] =
                TransformLocalToParent(pos, rot, cd_data->shape_data.triangle_rigid[start + 1]);
            cd_data->shape_data.triangle_global[start + 2] =
                TransformLocalToParent(pos, rot, cd_data->shape_data.triangle_rigid[start + 2]);
        }
        cd_data->shape_data.obj_data_R_global[index] = Mult(rot, obj_data_R[index]);
    }
}

// -----------------------------------------------------------------------------

void ChNarrowphase::Dispatch_Init(uint index,
                                  uint& icoll,
                                  uint& ID_A,
                                  uint& ID_B,
                                  ConvexShape* shapeA,
                                  ConvexShape* shapeB) {
    const std::vector<uint>& obj_data_ID = cd_data->shape_data.id_rigid;
    const std::vector<long long>& pair_shapeIDs = cd_data->pair_shapeIDs;

    // Unpack the identifiers for the two shapes involved in this collision
    long long p = pair_shapeIDs[index];
    vec2 pair = I2(int(p >> 32), int(p & 0xffffffff));

    ID_A = obj_data_ID[pair.x];
    ID_B = obj_data_ID[pair.y];  // Get the identifiers of the two associated objects (bodies)

    shapeA->index = pair.x;
    shapeB->index = pair.y;

    shapeA->data = &cd_data->shape_data;
    shapeB->data = &cd_data->shape_data;

    //// TODO: what is the best way to dispatch this?
    icoll = contact_index[index];
}

void ChNarrowphase::Dispatch_Finalize(uint icoll, uint ID_A, uint ID_B, int nC) {
    std::vector<vec2>& body_ids = cd_data->bids_rigid_rigid;

    // Mark the active contacts and set their body IDs
    for (int i = 0; i < nC; i++) {
        contact_rigid_active[icoll + i] = true;
        body_ids[icoll + i] = I2(ID_A, ID_B);
    }
}

void ChNarrowphase::DispatchMPR() {
    const real envelope = cd_data->collision_envelope;
    std::vector<real3>& norm = cd_data->norm_rigid_rigid;
    std::vector<real3>& ptA = cd_data->cpta_rigid_rigid;
    std::vector<real3>& ptB = cd_data->cptb_rigid_rigid;
    std::vector<real>& contactDepth = cd_data->dpth_rigid_rigid;
    std::vector<real>& effective_radius = cd_data->erad_rigid_rigid;

    ConvexShape shapeA;
    ConvexShape shapeB;

    double default_eff_radius = ChCollisionInfo::GetDefaultEffectiveCurvatureRadius();

#pragma omp parallel for private(shapeA, shapeB)
    for (int index = 0; index < (signed)num_potential_rigid_contacts; index++) {
        uint ID_A, ID_B, icoll;

        Dispatch_Init(index, icoll, ID_A, ID_B, &shapeA, &shapeB);

        if (MPRCollision(&shapeA, &shapeB, envelope, norm[icoll], ptA[icoll], ptB[icoll], contactDepth[icoll])) {
            effective_radius[icoll] = default_eff_radius;
            // The number of contacts reported by MPR is always 1.
            Dispatch_Finalize(icoll, ID_A, ID_B, 1);
        }
    }
}

void ChNarrowphase::DispatchPRIMS() {
    const real envelope = cd_data->collision_envelope;
    real3* norm = cd_data->norm_rigid_rigid.data();
    real3* ptA = cd_data->cpta_rigid_rigid.data();
    real3* ptB = cd_data->cptb_rigid_rigid.data();
    real* contactDepth = cd_data->dpth_rigid_rigid.data();
    real* effective_radius = cd_data->erad_rigid_rigid.data();

    ConvexShape shapeA;
    ConvexShape shapeB;

#pragma omp parallel for private(shapeA, shapeB)
    for (int index = 0; index < (signed)num_potential_rigid_contacts; index++) {
        uint ID_A, ID_B, icoll;

        int nC;

        Dispatch_Init(index, icoll, ID_A, ID_B, &shapeA, &shapeB);

        if (PRIMSCollision(&shapeA, &shapeB, 2 * envelope, &norm[icoll], &ptA[icoll], &ptB[icoll], &contactDepth[icoll],
                           &effective_radius[icoll], nC)) {
            Dispatch_Finalize(icoll, ID_A, ID_B, nC);
        }
    }
}

void ChNarrowphase::DispatchHybridMPR() {
    const real envelope = cd_data->collision_envelope;
    real3* norm = cd_data->norm_rigid_rigid.data();
    real3* ptA = cd_data->cpta_rigid_rigid.data();
    real3* ptB = cd_data->cptb_rigid_rigid.data();
    real* contactDepth = cd_data->dpth_rigid_rigid.data();
    real* effective_radius = cd_data->erad_rigid_rigid.data();

    ConvexShape shapeA;
    ConvexShape shapeB;

    double default_eff_radius = ChCollisionInfo::GetDefaultEffectiveCurvatureRadius();

#pragma omp parallel for private(shapeA, shapeB)
    for (int index = 0; index < (signed)num_potential_rigid_contacts; index++) {
        uint ID_A, ID_B, icoll;

        int nC;

        Dispatch_Init(index, icoll, ID_A, ID_B, &shapeA, &shapeB);

        if (PRIMSCollision(&shapeA, &shapeB, 2 * envelope, &norm[icoll], &ptA[icoll], &ptB[icoll], &contactDepth[icoll],
                           &effective_radius[icoll], nC)) {
            Dispatch_Finalize(icoll, ID_A, ID_B, nC);
        } else if (MPRCollision(&shapeA, &shapeB, envelope, norm[icoll], ptA[icoll], ptB[icoll], contactDepth[icoll])) {
            effective_radius[icoll] = default_eff_radius;
            Dispatch_Finalize(icoll, ID_A, ID_B, 1);
        }
        // delete shapeA;
        // delete shapeB;
    }
}

// -----------------------------------------------------------------------------

void ChNarrowphase::ProcessRigidRigid() {
    std::vector<real3>& norm_data = cd_data->norm_rigid_rigid;
    std::vector<real3>& cpta_data = cd_data->cpta_rigid_rigid;
    std::vector<real3>& cptb_data = cd_data->cptb_rigid_rigid;
    std::vector<real>& dpth_data = cd_data->dpth_rigid_rigid;
    std::vector<real>& erad_data = cd_data->erad_rigid_rigid;
    std::vector<vec2>& bids_data = cd_data->bids_rigid_rigid;
    std::vector<long long>& contact_shapeIDs = cd_data->contact_shapeIDs;
    uint& num_rigid_contacts = cd_data->num_rigid_contacts;

    // Set maximum possible number of contacts for each potential collision
    // (depending on the narrowphase algorithm and on the types of shapes in
    // potential collision) and calculate the total number of potential contacts.
    int num_potentialContacts = PreprocessCount();

    // Create storage to hold maximum number of contacts in worse case
    norm_data.resize(num_potentialContacts);
    cpta_data.resize(num_potentialContacts);
    cptb_data.resize(num_potentialContacts);
    dpth_data.resize(num_potentialContacts);
    erad_data.resize(num_potentialContacts);
    bids_data.resize(num_potentialContacts);

    // These flags will keep track of which potential contacts are actually active
    // (as decided by the narrowphase algorithm).
    contact_rigid_active.resize(num_potentialContacts);
    thrust::fill(contact_rigid_active.begin(), contact_rigid_active.end(), false);

    switch (algorithm) {
        case Algorithm::MPR:
            DispatchMPR();
            break;
        case Algorithm::PRIMS:
            DispatchPRIMS();
            break;
        case Algorithm::HYBRID:
            DispatchHybridMPR();
            break;
    }

    // Calculate total number of actual (active) contacts
    num_rigid_contacts = (uint)Thrust_Count(contact_rigid_active, 1);

    // Remove elements corresponding to inactive contacts. We do this in one step,
    // using zip iterators and removing all entries for which contact_active is 'false'.
    thrust::remove_if(
        THRUST_PAR thrust::make_zip_iterator(thrust::make_tuple(norm_data.begin(), cpta_data.begin(), cptb_data.begin(),
                                                                dpth_data.begin(), erad_data.begin(), bids_data.begin(),
                                                                contact_shapeIDs.begin())),
        thrust::make_zip_iterator(thrust::make_tuple(norm_data.end(), cpta_data.end(), cptb_data.end(), dpth_data.end(),
                                                     erad_data.end(), bids_data.end(), contact_shapeIDs.end())),
        contact_rigid_active.begin(), thrust::logical_not<bool>());

    // Resize all lists so that we don't access invalid contacts
    norm_data.resize(num_rigid_contacts);
    cpta_data.resize(num_rigid_contacts);
    cptb_data.resize(num_rigid_contacts);
    dpth_data.resize(num_rigid_contacts);
    erad_data.resize(num_rigid_contacts);
    bids_data.resize(num_rigid_contacts);
    contact_shapeIDs.resize(num_rigid_contacts);
}

// -----------------------------------------------------------------------------

inline int GridCoord(real x, real inv_bin_edge, real minimum) {
    real l = x - minimum;
    int c = (int)Round(l * inv_bin_edge);
    return c;
}

inline int GridHash(int x, int y, int z, const vec3& bins_per_axis) {
    return ((z * bins_per_axis.y) * bins_per_axis.x) + (y * bins_per_axis.x) + x;
}

void ChNarrowphase::ProcessFluid() {
    // Readability replacements
    const int num_fluid_bodies = cd_data->state_data.num_fluid_bodies;
    if (num_fluid_bodies == 0)
        return;

    const real radius = cd_data->p_kernel_radius + cd_data->p_collision_envelope;
    const real collision_envelope = cd_data->p_collision_envelope;
    const real3& min_bounding_point = cd_data->ff_min_bounding_point;
    const real3& max_bounding_point = cd_data->ff_max_bounding_point;

    const std::vector<real3>& pos_fluid = *cd_data->state_data.pos_3dof;
    std::vector<real3>& sorted_pos_fluid = *cd_data->state_data.sorted_pos_3dof;

    std::vector<int>& neighbor_fluid_fluid = cd_data->neighbor_3dof_3dof;
    std::vector<int>& contact_counts = cd_data->c_counts_3dof_3dof;
    std::vector<int>& particle_indices = cd_data->particle_indices_3dof;
    std::vector<int>& reverse_mapping = cd_data->reverse_mapping_3dof;
    vec3& bins_per_axis = cd_data->ff_bins_per_axis;
    uint& num_fluid_contacts = cd_data->num_fluid_contacts;

    const real radius_envelope = radius + collision_envelope;
    const real radius_squared = radius_envelope * radius_envelope;

    real3 diag = max_bounding_point - min_bounding_point;
    bins_per_axis = vec3(diag / (radius_envelope * 2));
    real inv_bin_edge = real(1.0) / (radius_envelope * 2);
    size_t grid_size = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;

    //====================================
    neighbor_fluid_fluid.resize(num_fluid_bodies * max_neighbors);
    contact_counts.resize(num_fluid_bodies);
    particle_indices.resize(num_fluid_bodies);
    reverse_mapping.resize(num_fluid_bodies);
    ff_bin_ids.resize(num_fluid_bodies);
    //====================================
    sorted_pos_fluid.resize(num_fluid_bodies);
    //====================================
    ff_bin_starts.resize(grid_size);
    ff_bin_ends.resize(grid_size);
    //====================================
    Thrust_Fill(ff_bin_starts, 0);
    Thrust_Fill(ff_bin_ends, 0);
    Thrust_Fill(contact_counts, 0);
    Thrust_Fill(neighbor_fluid_fluid, 0);
    //====================================

#pragma omp parallel for
    for (int i = 0; i < num_fluid_bodies; i++) {
        real3 p = pos_fluid[i];
        ff_bin_ids[i] = GridHash(GridCoord(p.x, inv_bin_edge, min_bounding_point.x),
                                 GridCoord(p.y, inv_bin_edge, min_bounding_point.y),
                                 GridCoord(p.z, inv_bin_edge, min_bounding_point.z), bins_per_axis);
        particle_indices[i] = i;
    }

    Thrust_Sort_By_Key(ff_bin_ids, particle_indices);

#pragma omp parallel for
    for (int i = 0; i < num_fluid_bodies; i++) {
        int index = particle_indices[i];
        sorted_pos_fluid[i] = pos_fluid[index];

        reverse_mapping[index] = i;

        int c = ff_bin_ids[i];
        if (i == 0) {
            ff_bin_starts[c] = i;
        } else {
            int p = ff_bin_ids[i - 1];
            if (c != p) {
                ff_bin_starts[c] = i;
                ff_bin_ends[p] = i;
            }
        }
        if (i == num_fluid_bodies - 1) {
            ff_bin_ends[c] = i + 1;
        }
    }

#pragma omp parallel for
    for (int p = 0; p < num_fluid_bodies; p++) {
        real3 xi = sorted_pos_fluid[p];
        const int cx = GridCoord(xi.x, inv_bin_edge, min_bounding_point.x);
        const int cy = GridCoord(xi.y, inv_bin_edge, min_bounding_point.y);
        const int cz = GridCoord(xi.z, inv_bin_edge, min_bounding_point.z);

        int contact_count = 0;
        for (int k = cz - 1; k <= cz + 1; ++k) {
            for (int j = cy - 1; j <= cy + 1; ++j) {
                for (int i = cx - 1; i <= cx + 1; ++i) {
                    const int cellIndex = GridHash(i, j, k, bins_per_axis);
                    const int cellStart = ff_bin_starts[cellIndex];
                    const int cellEnd = ff_bin_ends[cellIndex];
                    for (int q = cellStart; q < cellEnd; ++q) {
                        // if (q == p) { continue; }  // disabled this so that we get self contact
                        const real3 xj = sorted_pos_fluid[q];
                        const real3 xij = xi - xj;
                        if (Dot(xij) < radius_squared) {
                            if (contact_count < max_neighbors) {
                                neighbor_fluid_fluid[p * max_neighbors + contact_count] = q;
                                ++contact_count;
                            }
                        }
                    }
                }
            }
        }
        contact_counts[p] = contact_count;
    }

    num_fluid_contacts = Thrust_Total(contact_counts);
}

// -----------------------------------------------------------------------------

void ChNarrowphase::ProcessRigidFluid() {
    // Readability replacements
    const real sphere_radius = cd_data->p_kernel_radius;
    const int num_spheres = cd_data->state_data.num_fluid_bodies;
    const std::vector<real3>& pos_spheres = *cd_data->state_data.sorted_pos_3dof;
    const short2& family = cd_data->p_collision_family;
    const real envelope = cd_data->collision_envelope;

    std::vector<real3>& norm_rigid_sphere = cd_data->norm_rigid_fluid;
    std::vector<real3>& cpta_rigid_sphere = cd_data->cpta_rigid_fluid;
    std::vector<real>& dpth_rigid_sphere = cd_data->dpth_rigid_fluid;
    std::vector<int>& neighbor_rigid_sphere = cd_data->neighbor_rigid_fluid;
    std::vector<int>& contact_counts = cd_data->c_counts_rigid_fluid;
    uint& num_contacts = cd_data->num_rigid_fluid_contacts;

    const vec3& bins_per_axis = cd_data->bins_per_axis;
    real3 global_origin = cd_data->global_origin;
    real3 inv_bin_size = cd_data->inv_bin_size;
    const std::vector<short2>& fam_data = cd_data->shape_data.fam_rigid;
    const real radius = sphere_radius;

    uint total_bins = (bins_per_axis.x + 1) * (bins_per_axis.y + 1) * (bins_per_axis.z + 1);
    is_rigid_bin_active.resize(total_bins);

    Thrust_Fill(is_rigid_bin_active, 1000000000);
#pragma omp parallel for
    for (int index = 0; index < (signed)cd_data->num_active_bins; index++) {
        uint bin_number = cd_data->bin_active[index];
        if (bin_number < total_bins) {
            // printf("bin_number: %d\n", index, bin_number);
            is_rigid_bin_active[bin_number] = index;
        }
    }
    f_bin_intersections.resize(num_spheres + 1);
    f_bin_intersections[num_spheres] = 0;

#pragma omp parallel for
    for (int p = 0; p < num_spheres; p++) {
        real3 pos_sphere = pos_spheres[p];
        vec3 gmin = HashMin(pos_sphere - real3(radius + envelope) - global_origin, inv_bin_size);
        vec3 gmax = HashMax(pos_sphere + real3(radius + envelope) - global_origin, inv_bin_size);
        f_bin_intersections[p] = (gmax.x - gmin.x + 1) * (gmax.y - gmin.y + 1) * (gmax.z - gmin.z + 1);
    }

    Thrust_Exclusive_Scan(f_bin_intersections);
    uint f_number_of_bin_intersections = f_bin_intersections.back();

    f_bin_number.resize(f_number_of_bin_intersections);
    f_bin_number_out.resize(f_number_of_bin_intersections);
    f_bin_fluid_number.resize(f_number_of_bin_intersections);
    f_bin_start_index.resize(f_number_of_bin_intersections);

#pragma omp parallel for
    for (int p = 0; p < num_spheres; p++) {
        uint count = 0, i, j, k;
        real3 pos_sphere = pos_spheres[p];
        vec3 gmin = HashMin(pos_sphere - real3(radius + envelope) - global_origin, inv_bin_size);
        vec3 gmax = HashMax(pos_sphere + real3(radius + envelope) - global_origin, inv_bin_size);
        uint mInd = f_bin_intersections[p];
        for (i = (unsigned)gmin.x; i <= (unsigned)gmax.x; i++) {
            for (j = (unsigned)gmin.y; j <= (unsigned)gmax.y; j++) {
                for (k = (unsigned)gmin.z; k <= (unsigned)gmax.z; k++) {
                    f_bin_number[mInd + count] = Hash_Index(vec3(i, j, k), bins_per_axis);
                    f_bin_fluid_number[mInd + count] = p;
                    count++;
                }
            }
        }
    }

    Thrust_Sort_By_Key(f_bin_number, f_bin_fluid_number);
    auto f_number_of_bins_active = (uint)(Run_Length_Encode(f_bin_number, f_bin_number_out, f_bin_start_index));

    f_bin_start_index.resize(f_number_of_bins_active + 1);
    f_bin_start_index[f_number_of_bins_active] = 0;
    Thrust_Exclusive_Scan(f_bin_start_index);
    std::vector<uint> f_bin_num_contact(f_number_of_bins_active + 1);
    f_bin_num_contact[f_number_of_bins_active] = 0;

    norm_rigid_sphere.resize(num_spheres * max_rigid_neighbors);
    cpta_rigid_sphere.resize(num_spheres * max_rigid_neighbors);
    dpth_rigid_sphere.resize(num_spheres * max_rigid_neighbors);
    neighbor_rigid_sphere.resize(num_spheres * max_rigid_neighbors);
    contact_counts.resize(num_spheres + 1);

    Thrust_Fill(contact_counts, 0);
    // For each rigid bin
    for (int index = 0; index < (signed)f_number_of_bins_active; index++) {
        uint bin_number = f_bin_number_out[index];
        uint rigid_index = is_rigid_bin_active[bin_number];
        // check if the bin is active
        if (rigid_index != 1000000000) {
            // start and end of fluid in this bin
            uint start = f_bin_start_index[index];
            uint end = f_bin_start_index[index + 1];
            // start and end of rigid bodies in this bin
            uint rigid_start = cd_data->bin_start_index[rigid_index];
            uint rigid_end = cd_data->bin_start_index[rigid_index + 1];
#pragma omp parallel for
            for (int i = start; i < (signed)end; i++) {
                uint p = f_bin_fluid_number[i];
                real3 pos_sphere = pos_spheres[p];
                real3 Bmin = pos_sphere - real3(radius + envelope) - global_origin;
                real3 Bmax = pos_sphere + real3(radius + envelope) - global_origin;
                ConvexShapeSphere* shapeB = new ConvexShapeSphere(pos_sphere, sphere_radius * .5);

                for (uint j = rigid_start; j < rigid_end; j++) {
                    if (contact_counts[p] < max_rigid_neighbors) {
                        uint shape_id_a = cd_data->bin_aabb_number[j];
                        real3 Amin = cd_data->aabb_min[shape_id_a];
                        real3 Amax = cd_data->aabb_max[shape_id_a];
                        // if the sphere and the rigid body appear in the same bin more than once, dont count
                        if (current_bin(Amin, Amax, Bmin, Bmax, inv_bin_size, bins_per_axis, bin_number) == true) {
                            if (overlap(Amin, Amax, Bmin, Bmax) && collide(family, fam_data[shape_id_a])) {
                                ConvexShape* shapeA = new ConvexShape(shape_id_a, &cd_data->shape_data);
                                real3 ptA, ptB, norm;
                                real depth, erad = 0;
                                int nC = 0;
                                if (PRIMSCollision(shapeA, shapeB, 2 * envelope, &norm, &ptA, &ptB, &depth, &erad,
                                                   nC)) {
                                    if (nC == 1) {
                                        uint bodyA = cd_data->shape_data.id_rigid[shape_id_a];
                                        neighbor_rigid_sphere[p * max_rigid_neighbors + contact_counts[p]] = bodyA;
                                        norm_rigid_sphere[p * max_rigid_neighbors + contact_counts[p]] = norm;
                                        cpta_rigid_sphere[p * max_rigid_neighbors + contact_counts[p]] = ptA;
                                        dpth_rigid_sphere[p * max_rigid_neighbors + contact_counts[p]] = depth;
                                        contact_counts[p]++;
                                    }
                                } else if (MPRCollision(
                                               shapeA, shapeB, envelope,
                                               norm_rigid_sphere[p * max_rigid_neighbors + contact_counts[p]],
                                               cpta_rigid_sphere[p * max_rigid_neighbors + contact_counts[p]], ptB,
                                               dpth_rigid_sphere[p * max_rigid_neighbors + contact_counts[p]])) {
                                    uint bodyA = cd_data->shape_data.id_rigid[shape_id_a];
                                    neighbor_rigid_sphere[p * max_rigid_neighbors + contact_counts[p]] = bodyA;
                                    contact_counts[p]++;
                                }
                                delete shapeA;
                            }
                        }
                    }
                }
                delete shapeB;
            }
        }
    }

    Thrust_Exclusive_Scan(contact_counts);
    num_contacts = contact_counts[num_spheres];
}

}  // end namespace chrono
