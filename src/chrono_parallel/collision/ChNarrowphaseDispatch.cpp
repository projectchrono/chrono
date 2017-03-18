#include <algorithm>

#include "chrono/collision/ChCCollisionModel.h"

#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/collision/ChCollision.h"
#include "chrono_parallel/collision/ChNarrowphaseUtils.h"
#include "chrono_parallel/collision/ChBroadphaseUtils.h"
#include "chrono_parallel/collision/ChNarrowphaseMPR.h"
#include "chrono_parallel/collision/ChNarrowphaseR.h"

#include "chrono_parallel/physics/Ch3DOFContainer.h"

#include <thrust/remove.h>
#include <thrust/sort.h>
#include <thrust/transform_reduce.h>
#include <thrust/count.h>
#include <thrust/iterator/constant_iterator.h>

#if defined(CHRONO_OPENMP_ENABLED)
#include <thrust/system/omp/execution_policy.h>
#elif defined(CHRONO_TBB_ENABLED)
#include <thrust/system/tbb/execution_policy.h>
#endif

namespace chrono {
namespace collision {

void ChCNarrowphaseDispatch::ClearContacts() {
    // Return now if no potential collisions.
    if (num_potential_rigid_contacts == 0) {
        data_manager->host_data.norm_rigid_rigid.resize(0);
        data_manager->host_data.cpta_rigid_rigid.resize(0);
        data_manager->host_data.cptb_rigid_rigid.resize(0);
        data_manager->host_data.dpth_rigid_rigid.resize(0);
        data_manager->host_data.erad_rigid_rigid.resize(0);
        data_manager->host_data.bids_rigid_rigid.resize(0);
    }
}

void ChCNarrowphaseDispatch::ProcessRigids() {
    //======== Indexing variables and other information
    num_potential_rigid_contacts = data_manager->num_rigid_contacts;
    num_potential_rigid_fluid_contacts = data_manager->num_rigid_fluid_contacts;
    num_potential_fluid_contacts = data_manager->num_fluid_contacts;
    narrowphase_algorithm = data_manager->settings.collision.narrowphase_algorithm;
    collision_envelope = data_manager->settings.collision.collision_envelope;
    ClearContacts();
    // Transform Rigid body shapes to global coordinate system
    PreprocessLocalToParent();

    if (num_potential_rigid_contacts != 0) {
        DispatchRigid();
    }
    if (data_manager->num_fluid_bodies != 0) {
        DispatchRigidFluid();
    }
    if (data_manager->num_fea_tets != 0) {
        DispatchRigidTet();
    }
}

void ChCNarrowphaseDispatch::PreprocessCount() {
    // MPR always reports at most one contact per pair.
    if (narrowphase_algorithm == NarrowPhaseType::NARROWPHASE_MPR) {
        thrust::fill(contact_index.begin(), contact_index.end(), 1);
        return;
    }

    // NarrowphaseR (and hence the hybrid algorithms) may produce different number
    // of contacts per pair, depending on the interacting shapes:
    //   - an interaction involving a sphere can produce at most one contact
    //   - an interaction involving a capsule can produce up to two contacts
    //   - a box-box interaction can produce up to 8 contacts

    // shape type (per shape)
    const shape_type* obj_data_T = data_manager->shape_data.typ_rigid.data();
    // encoded shape IDs (per collision pair)
    const long long* collision_pair = data_manager->host_data.contact_pairs.data();

#pragma omp parallel for
    for (int index = 0; index < (signed)num_potential_rigid_contacts; index++) {
        // Identify the two candidate shapes and get their types.
        vec2 pair = I2(int(collision_pair[index] >> 32), int(collision_pair[index] & 0xffffffff));
        shape_type type1 = obj_data_T[pair.x];
        shape_type type2 = obj_data_T[pair.y];

        // Set the maximum number of possible contacts for this particular pair
        if (type1 == SPHERE || type2 == SPHERE) {
            contact_index[index] = 1;
        } else if (type1 == CAPSULE || type2 == CAPSULE) {
            contact_index[index] = 2;
            ////} else if (type1 == BOX && type2 == BOX) {
            ////  contact_index[index] = 8;
        } else {
            contact_index[index] = 1;
        }
    }
}

void ChCNarrowphaseDispatch::PreprocessLocalToParent() {
    LOG(TRACE) << "ChCNarrowphaseDispatch::PreprocessLocalToParent()";
    uint num_shapes = data_manager->num_rigid_shapes;

    const custom_vector<int>& obj_data_T = data_manager->shape_data.typ_rigid;
    const custom_vector<real3>& obj_data_A = data_manager->shape_data.ObA_rigid;
    const custom_vector<quaternion>& obj_data_R = data_manager->shape_data.ObR_rigid;
    const custom_vector<uint>& obj_data_ID = data_manager->shape_data.id_rigid;

    const custom_vector<real3>& body_pos = data_manager->host_data.pos_rigid;
    const custom_vector<quaternion>& body_rot = data_manager->host_data.rot_rigid;

    data_manager->shape_data.obj_data_A_global.resize(num_shapes);

    data_manager->shape_data.obj_data_R_global.resize(num_shapes);
    data_manager->shape_data.triangle_global.resize(data_manager->shape_data.triangle_rigid.size());

#pragma omp parallel for
    for (int index = 0; index < (signed)num_shapes; index++) {
        shape_type T = obj_data_T[index];

        // Get the identifier for the object associated with this collision shape
        uint ID = obj_data_ID[index];

        real3 pos = body_pos[ID];       // Get the global object position
        quaternion rot = body_rot[ID];  // Get the global object rotation

        data_manager->shape_data.obj_data_A_global[index] = TransformLocalToParent(pos, rot, obj_data_A[index]);
        if (T == TRIANGLEMESH) {
            int start = data_manager->shape_data.start_rigid[index];
            data_manager->shape_data.triangle_global[start + 0] =
                TransformLocalToParent(pos, rot, data_manager->shape_data.triangle_rigid[start + 0]);
            data_manager->shape_data.triangle_global[start + 1] =
                TransformLocalToParent(pos, rot, data_manager->shape_data.triangle_rigid[start + 1]);
            data_manager->shape_data.triangle_global[start + 2] =
                TransformLocalToParent(pos, rot, data_manager->shape_data.triangle_rigid[start + 2]);
        }
        data_manager->shape_data.obj_data_R_global[index] = Mult(rot, obj_data_R[index]);
    }
}

void ChCNarrowphaseDispatch::Dispatch_Init(uint index,
                                           uint& icoll,
                                           uint& ID_A,
                                           uint& ID_B,
                                           ConvexShape* shapeA,
                                           ConvexShape* shapeB) {
    const custom_vector<uint>& obj_data_ID = data_manager->shape_data.id_rigid;
    const custom_vector<long long>& contact_pair = data_manager->host_data.contact_pairs;
    real3* convex_data = data_manager->shape_data.convex_rigid.data();

    long long p = contact_pair[index];
    vec2 pair =
        I2(int(p >> 32), int(p & 0xffffffff));  // Get the identifiers for the two shapes involved in this collision

    ID_A = obj_data_ID[pair.x];
    ID_B = obj_data_ID[pair.y];  // Get the identifiers of the two associated objects (bodies)

    shapeA->index = pair.x;
    shapeB->index = pair.y;

    shapeA->data = &data_manager->shape_data;
    shapeB->data = &data_manager->shape_data;

    //// TODO: what is the best way to dispatch this?
    icoll = contact_index[index];
}

void ChCNarrowphaseDispatch::Dispatch_Finalize(uint icoll, uint ID_A, uint ID_B, int nC) {
    custom_vector<vec2>& body_ids = data_manager->host_data.bids_rigid_rigid;

    // Mark the active contacts and set their body IDs
    for (int i = 0; i < nC; i++) {
        contact_rigid_active[icoll + i] = true;
        body_ids[icoll + i] = I2(ID_A, ID_B);
    }
}

void ChCNarrowphaseDispatch::DispatchMPR() {
    custom_vector<real3>& norm = data_manager->host_data.norm_rigid_rigid;
    custom_vector<real3>& ptA = data_manager->host_data.cpta_rigid_rigid;
    custom_vector<real3>& ptB = data_manager->host_data.cptb_rigid_rigid;
    custom_vector<real>& contactDepth = data_manager->host_data.dpth_rigid_rigid;
    custom_vector<real>& effective_radius = data_manager->host_data.erad_rigid_rigid;

    ConvexShape shapeA;
    ConvexShape shapeB;

#pragma omp parallel for private(shapeA, shapeB)
    for (int index = 0; index < (signed)num_potential_rigid_contacts; index++) {
        uint ID_A, ID_B, icoll;

        Dispatch_Init(index, icoll, ID_A, ID_B, &shapeA, &shapeB);

        if (MPRCollision(&shapeA, &shapeB, collision_envelope, norm[icoll], ptA[icoll], ptB[icoll],
                         contactDepth[icoll])) {
            effective_radius[icoll] = edge_radius;
            // The number of contacts reported by MPR is always 1.
            Dispatch_Finalize(icoll, ID_A, ID_B, 1);
        }
    }
}

void ChCNarrowphaseDispatch::DispatchR() {
    real3* norm = data_manager->host_data.norm_rigid_rigid.data();
    real3* ptA = data_manager->host_data.cpta_rigid_rigid.data();
    real3* ptB = data_manager->host_data.cptb_rigid_rigid.data();
    real* contactDepth = data_manager->host_data.dpth_rigid_rigid.data();
    real* effective_radius = data_manager->host_data.erad_rigid_rigid.data();

    ConvexShape shapeA;
    ConvexShape shapeB;

#pragma omp parallel for private(shapeA, shapeB)
    for (int index = 0; index < (signed)num_potential_rigid_contacts; index++) {
        uint ID_A, ID_B, icoll;

        int nC;

        Dispatch_Init(index, icoll, ID_A, ID_B, &shapeA, &shapeB);

        if (RCollision(&shapeA, &shapeB, 2 * collision_envelope, &norm[icoll], &ptA[icoll], &ptB[icoll],
                       &contactDepth[icoll], &effective_radius[icoll], nC)) {
            Dispatch_Finalize(icoll, ID_A, ID_B, nC);
        }
    }
}

void ChCNarrowphaseDispatch::DispatchHybridMPR() {
    real3* norm = data_manager->host_data.norm_rigid_rigid.data();
    real3* ptA = data_manager->host_data.cpta_rigid_rigid.data();
    real3* ptB = data_manager->host_data.cptb_rigid_rigid.data();
    real* contactDepth = data_manager->host_data.dpth_rigid_rigid.data();
    real* effective_radius = data_manager->host_data.erad_rigid_rigid.data();

    ConvexShape shapeA;
    ConvexShape shapeB;

#pragma omp parallel for private(shapeA, shapeB)
    for (int index = 0; index < (signed)num_potential_rigid_contacts; index++) {
        uint ID_A, ID_B, icoll;

        int nC;

        Dispatch_Init(index, icoll, ID_A, ID_B, &shapeA, &shapeB);

        if (RCollision(&shapeA, &shapeB, 2 * collision_envelope, &norm[icoll], &ptA[icoll], &ptB[icoll],
                       &contactDepth[icoll], &effective_radius[icoll], nC)) {
            Dispatch_Finalize(icoll, ID_A, ID_B, nC);
        } else if (MPRCollision(&shapeA, &shapeB, collision_envelope, norm[icoll], ptA[icoll], ptB[icoll],
                                contactDepth[icoll])) {
            effective_radius[icoll] = edge_radius;
            Dispatch_Finalize(icoll, ID_A, ID_B, 1);
        }
        // delete shapeA;
        // delete shapeB;
    }
}

void ChCNarrowphaseDispatch::DispatchRigid() {
    LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchRigid() S";
    custom_vector<real3>& norm_data = data_manager->host_data.norm_rigid_rigid;
    custom_vector<real3>& cpta_data = data_manager->host_data.cpta_rigid_rigid;
    custom_vector<real3>& cptb_data = data_manager->host_data.cptb_rigid_rigid;
    custom_vector<real>& dpth_data = data_manager->host_data.dpth_rigid_rigid;
    custom_vector<real>& erad_data = data_manager->host_data.erad_rigid_rigid;
    custom_vector<vec2>& bids_data = data_manager->host_data.bids_rigid_rigid;
    custom_vector<long long>& contact_pairs = data_manager->host_data.contact_pairs;
    uint& num_rigid_contacts = data_manager->num_rigid_contacts;
    // Set maximum possible number of contacts for each potential collision
    // (depending on the narrowphase algorithm and on the types of shapes in
    // potential collision)
    contact_index.resize(num_potential_rigid_contacts + 1);
    contact_index[num_potential_rigid_contacts] = 0;
    PreprocessCount();

    // Scan to find total number of potential contacts
    Thrust_Exclusive_Scan(contact_index);
    int num_potentialContacts = contact_index.back();

    // Create storage to hold maximum number of contacts in worse case
    norm_data.resize(num_potentialContacts);
    cpta_data.resize(num_potentialContacts);
    cptb_data.resize(num_potentialContacts);
    dpth_data.resize(num_potentialContacts);
    erad_data.resize(num_potentialContacts);
    bids_data.resize(num_potentialContacts);

    // These flags will keep track of which collision pairs are actually active
    // (as decided by the narrowphase algorithm).
    contact_rigid_active.resize(num_potentialContacts);
    thrust::fill(contact_rigid_active.begin(), contact_rigid_active.end(), false);

    switch (narrowphase_algorithm) {
        case NarrowPhaseType::NARROWPHASE_MPR:
            DispatchMPR();
            break;
        case NarrowPhaseType::NARROWPHASE_R:
            DispatchR();
            break;
        case NarrowPhaseType::NARROWPHASE_HYBRID_MPR:
            DispatchHybridMPR();
            break;
    }

    num_rigid_contacts = (uint)Thrust_Count(contact_rigid_active, 1);
    // Remove elements corresponding to inactive contacts. We do this in one step,
    // using zip iterators and removing all entries for which contact_active is 'false'.
    thrust::remove_if(
        thrust::make_zip_iterator(thrust::make_tuple(norm_data.begin(), cpta_data.begin(), cptb_data.begin(),
                                                     dpth_data.begin(), erad_data.begin(), bids_data.begin(),
                                                     contact_pairs.begin())),
        thrust::make_zip_iterator(thrust::make_tuple(norm_data.end(), cpta_data.end(), cptb_data.end(), dpth_data.end(),
                                                     erad_data.end(), bids_data.end(), contact_pairs.end())),
        contact_rigid_active.begin(), thrust::logical_not<bool>());

    // Resize all lists so that we don't access invalid contacts
    norm_data.resize(num_rigid_contacts);
    cpta_data.resize(num_rigid_contacts);
    cptb_data.resize(num_rigid_contacts);
    dpth_data.resize(num_rigid_contacts);
    erad_data.resize(num_rigid_contacts);
    bids_data.resize(num_rigid_contacts);
    contact_pairs.resize(num_rigid_contacts);
    LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchRigid() E " << num_rigid_contacts;
}

void ChCNarrowphaseDispatch::DispatchRigidFluid() {
    LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchRigidFluid() S";

    RigidSphereContact(data_manager->node_container->kernel_radius,   //
                       data_manager->num_fluid_bodies,                //
                       data_manager->host_data.sorted_pos_3dof,       //
                       data_manager->node_container->family,          //
                       data_manager->host_data.norm_rigid_fluid,      //
                       data_manager->host_data.cpta_rigid_fluid,      //
                       data_manager->host_data.dpth_rigid_fluid,      //
                       data_manager->host_data.neighbor_rigid_fluid,  //
                       data_manager->host_data.c_counts_rigid_fluid,  //
                       data_manager->num_rigid_fluid_contacts);

    LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchRigidFluid() E " << data_manager->num_rigid_fluid_contacts;
}

inline int GridCoord(real x, real inv_bin_edge, real minimum) {
    real l = x - minimum;
    int c = (int)Round(l * inv_bin_edge);
    return c;
}

inline int GridHash(int x, int y, int z, const vec3& bins_per_axis) {
    return ((z * bins_per_axis.y) * bins_per_axis.x) + (y * bins_per_axis.x) + x;
}

void ChCNarrowphaseDispatch::SphereSphereContact(const int num_fluid_bodies,
                                                 const int body_offset,
                                                 const real radius,
                                                 const real collision_envelope,
                                                 const real3& min_bounding_point,
                                                 const real3& max_bounding_point,
                                                 const custom_vector<real3>& pos_fluid,
                                                 const custom_vector<real3>& vel_fluid,
                                                 custom_vector<real3>& sorted_pos_fluid,
                                                 custom_vector<real3>& sorted_vel_fluid,
                                                 DynamicVector<real>& v,
                                                 custom_vector<int>& neighbor_fluid_fluid,
                                                 custom_vector<int>& contact_counts,
                                                 custom_vector<int>& particle_indices,
                                                 custom_vector<int>& reverse_mapping,
                                                 vec3& bins_per_axis,
                                                 uint& num_fluid_contacts) {
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
    sorted_vel_fluid.resize(num_fluid_bodies);
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
        sorted_vel_fluid[i] = vel_fluid[index];
        v[body_offset + i * 3 + 0] = vel_fluid[index].x;
        v[body_offset + i * 3 + 1] = vel_fluid[index].y;
        v[body_offset + i * 3 + 2] = vel_fluid[index].z;

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

//#pragma omp parallel for
//    for (int i = 0; i < num_fluid_bodies; i++) {
//
//    }

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

void ChCNarrowphaseDispatch::DispatchFluid() {
    if (data_manager->num_fluid_bodies == 0) {
        return;
    }
    LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchFluid() S";
    const custom_vector<real3>& pos_fluid = data_manager->host_data.pos_3dof;
    const real radius = data_manager->node_container->kernel_radius + data_manager->node_container->collision_envelope;

    real3& max_bounding_point = data_manager->measures.collision.ff_max_bounding_point;
    real3& min_bounding_point = data_manager->measures.collision.ff_min_bounding_point;

    SphereSphereContact(data_manager->num_fluid_bodies, data_manager->num_rigid_bodies * 6 + data_manager->num_shafts,
                        radius, data_manager->node_container->collision_envelope, min_bounding_point,
                        max_bounding_point, pos_fluid, data_manager->host_data.vel_3dof,
                        data_manager->host_data.sorted_pos_3dof, data_manager->host_data.sorted_vel_3dof,
                        data_manager->host_data.v, data_manager->host_data.neighbor_3dof_3dof,
                        data_manager->host_data.c_counts_3dof_3dof, data_manager->host_data.particle_indices_3dof,
                        data_manager->host_data.reverse_mapping_3dof, data_manager->measures.collision.ff_bins_per_axis,
                        data_manager->num_fluid_contacts);

    LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchFluid() E " << data_manager->num_fluid_contacts;
}

void ChCNarrowphaseDispatch::DispatchRigidTet() {
    // Fluid is in the fluid bins already
    // determine if fluid and tet bin are the same
    // collide tet with fluid
    if (data_manager->num_fluid_bodies > 0) {
        LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchMarkerTet() S";
        MarkerTetContact(data_manager->node_container->kernel_radius,  //
                         data_manager->num_fluid_bodies,               //
                         data_manager->host_data.sorted_pos_3dof,      //
                         data_manager->node_container->family,         //
                         data_manager->host_data.norm_marker_tet, data_manager->host_data.cptb_marker_tet,
                         data_manager->host_data.dpth_marker_tet, data_manager->host_data.neighbor_marker_tet,
                         data_manager->host_data.face_marker_tet, data_manager->host_data.c_counts_marker_tet,
                         data_manager->num_marker_tet_contacts);
        LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchMarkerTet() E " << data_manager->num_marker_tet_contacts;
    }
    LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchRigidTet() S";

    RigidTetContact(data_manager->host_data.norm_rigid_tet, data_manager->host_data.cpta_rigid_tet,
                    data_manager->host_data.cptb_rigid_tet, data_manager->host_data.dpth_rigid_tet,
                    data_manager->host_data.neighbor_rigid_tet, data_manager->host_data.face_rigid_tet,
                    data_manager->host_data.c_counts_rigid_tet, data_manager->num_rigid_tet_contacts);

    LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchRigidTet() E " << data_manager->num_rigid_tet_contacts;

    LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchRigidTetNode() S";
    RigidSphereContact(data_manager->fea_container->kernel_radius,       //
                       data_manager->num_fea_nodes,                      //
                       data_manager->host_data.pos_node_fea,             //
                       data_manager->fea_container->family,              //
                       data_manager->host_data.norm_rigid_tet_node,      //
                       data_manager->host_data.cpta_rigid_tet_node,      //
                       data_manager->host_data.dpth_rigid_tet_node,      //
                       data_manager->host_data.neighbor_rigid_tet_node,  //
                       data_manager->host_data.c_counts_rigid_tet_node,  //
                       data_manager->num_rigid_tet_node_contacts);
    LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchRigidTetNode() E " << data_manager->num_rigid_tet_node_contacts;
}
//==================================================================================================================================

void ChCNarrowphaseDispatch::RigidSphereContact(const real sphere_radius,
                                                const int num_spheres,
                                                const custom_vector<real3>& pos_spheres,
                                                const short2& family,
                                                custom_vector<real3>& norm_rigid_sphere,
                                                custom_vector<real3>& cpta_rigid_sphere,
                                                custom_vector<real>& dpth_rigid_sphere,
                                                custom_vector<int>& neighbor_rigid_sphere,
                                                custom_vector<int>& contact_counts,
                                                uint& num_contacts) {
    int num_rigid_shapes = data_manager->num_rigid_shapes;
    real3 global_origin = data_manager->measures.collision.global_origin;
    vec3 bins_per_axis = data_manager->settings.collision.bins_per_axis;
    real3 inv_bin_size = data_manager->measures.collision.inv_bin_size;
    const custom_vector<short2>& fam_data = data_manager->shape_data.fam_rigid;
    const real radius = sphere_radius;

    uint total_bins = (bins_per_axis.x + 1) * (bins_per_axis.y + 1) * (bins_per_axis.z + 1);
    is_rigid_bin_active.resize(total_bins);

    Thrust_Fill(is_rigid_bin_active, 1000000000);
#pragma omp parallel for
    for (int index = 0; index < (signed)data_manager->measures.collision.number_of_bins_active; index++) {
        uint bin_number = data_manager->host_data.bin_number_out[index];
        if (bin_number < total_bins) {
            // printf("bin_number: %d\n", index, bin_number);
            is_rigid_bin_active[bin_number] = index;
        }
    }
    f_bin_intersections.resize(num_spheres + 1);
    f_bin_intersections[num_spheres] = 0;
    LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchRigidSphere is_rigid_bin_active";
#pragma omp parallel for
    for (int p = 0; p < num_spheres; p++) {
        real3 pos_sphere = pos_spheres[p];
        vec3 gmin = HashMin(pos_sphere - real3(radius + collision_envelope) - global_origin, inv_bin_size);
        vec3 gmax = HashMax(pos_sphere + real3(radius + collision_envelope) - global_origin, inv_bin_size);
        f_bin_intersections[p] = (gmax.x - gmin.x + 1) * (gmax.y - gmin.y + 1) * (gmax.z - gmin.z + 1);
    }
    Thrust_Exclusive_Scan(f_bin_intersections);
    uint f_number_of_bin_intersections = f_bin_intersections.back();
    LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchRigidSphere Thrust_Exclusive_Scan " << f_number_of_bin_intersections;
    f_bin_number.resize(f_number_of_bin_intersections);
    f_bin_number_out.resize(f_number_of_bin_intersections);
    f_bin_fluid_number.resize(f_number_of_bin_intersections);
    f_bin_start_index.resize(f_number_of_bin_intersections);

#pragma omp parallel for
    for (int p = 0; p < num_spheres; p++) {
        uint count = 0, i, j, k;
        real3 pos_sphere = pos_spheres[p];
        vec3 gmin = HashMin(pos_sphere - real3(radius + collision_envelope) - global_origin, inv_bin_size);
        vec3 gmax = HashMax(pos_sphere + real3(radius + collision_envelope) - global_origin, inv_bin_size);
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
    LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchRigidSphere Hash";
    Thrust_Sort_By_Key(f_bin_number, f_bin_fluid_number);
    f_number_of_bins_active = (int)(Run_Length_Encode(f_bin_number, f_bin_number_out, f_bin_start_index));

    f_bin_start_index.resize(f_number_of_bins_active + 1);
    f_bin_start_index[f_number_of_bins_active] = 0;
    Thrust_Exclusive_Scan(f_bin_start_index);
    custom_vector<uint> f_bin_num_contact(f_number_of_bins_active + 1);
    f_bin_num_contact[f_number_of_bins_active] = 0;
    LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchRigidSphere Thrust_Exclusive_Scan 2 " << f_number_of_bins_active;
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
            uint rigid_start = data_manager->host_data.bin_start_index[rigid_index];
            uint rigid_end = data_manager->host_data.bin_start_index[rigid_index + 1];
#pragma omp parallel for
            for (int i = start; i < (signed)end; i++) {
                uint p = f_bin_fluid_number[i];
                real3 pos_sphere = pos_spheres[p];
                real3 Bmin = pos_sphere - real3(radius + collision_envelope) - global_origin;
                real3 Bmax = pos_sphere + real3(radius + collision_envelope) - global_origin;
                ConvexShapeSphere* shapeB = new ConvexShapeSphere(pos_sphere, sphere_radius * .5);

                for (uint j = rigid_start; j < rigid_end; j++) {
                    if (contact_counts[p] < max_rigid_neighbors) {
                        uint shape_id_a = data_manager->host_data.bin_aabb_number[j];
                        real3 Amin = data_manager->host_data.aabb_min[shape_id_a];
                        real3 Amax = data_manager->host_data.aabb_max[shape_id_a];
                        // if the sphere and the rigid body appear in the same bin more than once, dont count
                        if (current_bin(Amin, Amax, Bmin, Bmax, inv_bin_size, bins_per_axis, bin_number) == true) {
                            if (overlap(Amin, Amax, Bmin, Bmax) && collide(family, fam_data[shape_id_a])) {
                                ConvexShape* shapeA = new ConvexShape(shape_id_a, &data_manager->shape_data);
                                real3 ptA, ptB, norm;
                                real depth, erad = 0;
                                int nC = 0;
                                if (RCollision(shapeA, shapeB, 2 * collision_envelope, &norm, &ptA, &ptB, &depth, &erad,
                                               nC)) {
                                    if (nC == 1) {
                                        uint bodyA = data_manager->shape_data.id_rigid[shape_id_a];
                                        neighbor_rigid_sphere[p * max_rigid_neighbors + contact_counts[p]] = bodyA;
                                        norm_rigid_sphere[p * max_rigid_neighbors + contact_counts[p]] = norm;
                                        cpta_rigid_sphere[p * max_rigid_neighbors + contact_counts[p]] = ptA;
                                        dpth_rigid_sphere[p * max_rigid_neighbors + contact_counts[p]] = depth;
                                        contact_counts[p]++;
                                    }
                                } else if (MPRCollision(
                                               shapeA, shapeB, collision_envelope,
                                               norm_rigid_sphere[p * max_rigid_neighbors + contact_counts[p]],
                                               cpta_rigid_sphere[p * max_rigid_neighbors + contact_counts[p]], ptB,
                                               dpth_rigid_sphere[p * max_rigid_neighbors + contact_counts[p]])) {
                                    uint bodyA = data_manager->shape_data.id_rigid[shape_id_a];
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
    LOG(TRACE) << "ChCNarrowphaseDispatch::DispatchRigidSphere Narrowphase";
    Thrust_Exclusive_Scan(contact_counts);
    num_contacts = contact_counts[num_spheres];
}
//==================================================================================================================================

void ChCNarrowphaseDispatch::RigidTetContact(custom_vector<real3>& norm_rigid_tet,
                                             custom_vector<real3>& cpta_rigid_tet,
                                             custom_vector<real3>& cptb_rigid_tet,
                                             custom_vector<real>& dpth_rigid_tet,
                                             custom_vector<int>& neighbor_rigid_tet,
                                             custom_vector<real4>& face_rigid_tet,
                                             custom_vector<int>& contact_counts,
                                             uint& num_contacts) {
    int num_rigid_shapes = data_manager->num_rigid_shapes;
    vec3 bins_per_axis = data_manager->settings.collision.bins_per_axis;
    real3 inv_bin_size = data_manager->measures.collision.inv_bin_size;

    int num_tets = (int)data_manager->host_data.boundary_element_fea.size();
    custom_vector<real3>& aabb_min_tet = data_manager->host_data.aabb_min_tet;
    custom_vector<real3>& aabb_max_tet = data_manager->host_data.aabb_max_tet;
    const custom_vector<short2>& fam_data = data_manager->shape_data.fam_rigid;
    uint total_bins = (bins_per_axis.x + 1) * (bins_per_axis.y + 1) * (bins_per_axis.z + 1);
    is_rigid_bin_active.resize(total_bins);
    Thrust_Fill(is_rigid_bin_active, 1000000000);
#pragma omp parallel for
    for (int index = 0; index < (signed)data_manager->measures.collision.number_of_bins_active; index++) {
        uint bin_number = data_manager->host_data.bin_number_out[index];
        if (bin_number < total_bins) {
            // printf("bin_number: %d\n", index, bin_number);
            is_rigid_bin_active[bin_number] = index;
        }
    }
    t_bin_intersections.resize(num_tets + 1);
    t_bin_intersections[num_tets] = 0;
#pragma omp parallel for
    for (int p = 0; p < num_tets; p++) {
        vec3 gmin = HashMin(aabb_min_tet[p], inv_bin_size);
        vec3 gmax = HashMax(aabb_max_tet[p], inv_bin_size);
        t_bin_intersections[p] = (gmax.x - gmin.x + 1) * (gmax.y - gmin.y + 1) * (gmax.z - gmin.z + 1);
    }
    Thrust_Exclusive_Scan(t_bin_intersections);
    uint t_number_of_bin_intersections = t_bin_intersections.back();

    t_bin_number.resize(t_number_of_bin_intersections);
    t_bin_number_out.resize(t_number_of_bin_intersections);
    t_bin_fluid_number.resize(t_number_of_bin_intersections);
    t_bin_start_index.resize(t_number_of_bin_intersections);

#pragma omp parallel for
    for (int p = 0; p < num_tets; p++) {
        uint count = 0, i, j, k;
        vec3 gmin = HashMin(aabb_min_tet[p], inv_bin_size);
        vec3 gmax = HashMax(aabb_max_tet[p], inv_bin_size);
        uint mInd = t_bin_intersections[p];
        for (i = (unsigned)gmin.x; i <= (unsigned)gmax.x; i++) {
            for (j = (unsigned)gmin.y; j <= (unsigned)gmax.y; j++) {
                for (k = (unsigned)gmin.z; k <= (unsigned)gmax.z; k++) {
                    t_bin_number[mInd + count] = Hash_Index(vec3(i, j, k), bins_per_axis);
                    t_bin_fluid_number[mInd + count] = p;
                    count++;
                }
            }
        }
    }
    Thrust_Sort_By_Key(t_bin_number, t_bin_fluid_number);
    uint t_number_of_bins_active = (int)(Run_Length_Encode(t_bin_number, t_bin_number_out, t_bin_start_index));

    t_bin_start_index.resize(t_number_of_bins_active + 1);
    t_bin_start_index[t_number_of_bins_active] = 0;
    Thrust_Exclusive_Scan(t_bin_start_index);
    custom_vector<uint> t_bin_num_contact(t_number_of_bins_active + 1);
    t_bin_num_contact[t_number_of_bins_active] = 0;

    norm_rigid_tet.resize(num_tets * max_rigid_neighbors);
    cpta_rigid_tet.resize(num_tets * max_rigid_neighbors);
    cptb_rigid_tet.resize(num_tets * max_rigid_neighbors);
    dpth_rigid_tet.resize(num_tets * max_rigid_neighbors);
    face_rigid_tet.resize(num_tets * max_rigid_neighbors);
    neighbor_rigid_tet.resize(num_tets * max_rigid_neighbors);
    contact_counts.resize(num_tets + 1);
    short2 family = data_manager->fea_container->family;
    Thrust_Fill(contact_counts, 0);

    for (int index = 0; index < (signed)t_number_of_bins_active; index++) {
        uint bin_number = t_bin_number_out[index];
        unsigned int rigid_index = is_rigid_bin_active[bin_number];
        if (rigid_index != 1000000000) {
            uint start = t_bin_start_index[index];
            uint end = t_bin_start_index[index + 1];
            uint rigid_start = data_manager->host_data.bin_start_index[rigid_index];
            uint rigid_end = data_manager->host_data.bin_start_index[rigid_index + 1];
#pragma omp parallel for
            for (int i = start; i < (signed)end; i++) {
                uint p = t_bin_fluid_number[i];
                real3 Bmin = aabb_min_tet[p];
                real3 Bmax = aabb_max_tet[p];

                uvec4 tet_index = data_manager->host_data.tet_indices[data_manager->host_data.boundary_element_fea[p]];
                real3* node_pos = data_manager->host_data.pos_node_fea.data();
                uvec4 bface = data_manager->host_data.boundary_triangles_fea[p];
                real3 t1 = node_pos[bface.x];
                real3 t2 = node_pos[bface.y];
                real3 t3 = node_pos[bface.z];
                uint bf = bface.w;
                ConvexShapeTetradhedron* shapeB = new ConvexShapeTetradhedron(tet_index, node_pos);
                for (uint j = rigid_start; j < rigid_end; j++) {
                    uint shape_id_a = data_manager->host_data.bin_aabb_number[j];
                    real3 Amin = data_manager->host_data.aabb_min[shape_id_a];
                    real3 Amax = data_manager->host_data.aabb_max[shape_id_a];
                    uint bodyA = data_manager->shape_data.id_rigid[shape_id_a];
                    if (current_bin(Amin, Amax, Bmin, Bmax, inv_bin_size, bins_per_axis, bin_number) == false) {
                        continue;
                    }
                    if (!overlap(Amin, Amax, Bmin, Bmax)) {
                        continue;
                    }
                    if (!collide(family, fam_data[shape_id_a]))
                        continue;
                    ConvexShape* shapeA = new ConvexShape(shape_id_a, &data_manager->shape_data);

                    real3 ptA, ptB, norm;
                    real depth;
                    real3 barycentric;
                    //int face;
                    real3 res;
                    if (MPRCollision(shapeA, shapeB, collision_envelope, norm, ptA, ptB, depth)) {
                        if (contact_counts[p] < max_rigid_neighbors) {
                            // FindTriIndex(ptB, tet_index, node_pos, face, barycentric);
                            // instead of finding the closest face, always use the surface face

                            SnapeToFaceBary(t1, t2, t3, ptB, res, barycentric);

                            norm_rigid_tet[p * max_rigid_neighbors + contact_counts[p]] = norm;
                            cpta_rigid_tet[p * max_rigid_neighbors + contact_counts[p]] = ptA;
                            cptb_rigid_tet[p * max_rigid_neighbors + contact_counts[p]] = ptB;
                            dpth_rigid_tet[p * max_rigid_neighbors + contact_counts[p]] = depth;
                            neighbor_rigid_tet[p * max_rigid_neighbors + contact_counts[p]] = bodyA;
                            face_rigid_tet[p * max_rigid_neighbors + contact_counts[p]] = real4(barycentric, bf);
                            contact_counts[p]++;
                        }
                    }
                    delete shapeA;
                }
                delete shapeB;
            }
        }
    }

    // Compute a mapping with weights for each vertex in contact (sum up weights)

    Thrust_Exclusive_Scan(contact_counts);
    num_contacts = contact_counts[num_tets];
}

void ChCNarrowphaseDispatch::MarkerTetContact(const real sphere_radius,
                                              const int num_spheres,
                                              const custom_vector<real3>& pos_spheres,
                                              const short2& family_sphere,
                                              custom_vector<real3>& norm_marker_tet,
                                              custom_vector<real3>& cptb_marker_tet,
                                              custom_vector<real>& dpth_marker_tet,
                                              custom_vector<int>& neighbor_marker_tet,
                                              custom_vector<real4>& face_marker_tet,
                                              custom_vector<int>& contact_counts,
                                              uint& num_contacts) {
    vec3 bins_per_axis = data_manager->settings.collision.bins_per_axis;
    real3 inv_bin_size = data_manager->measures.collision.inv_bin_size;
    real3 global_origin = data_manager->measures.collision.global_origin;
    int num_tets = (int)data_manager->host_data.boundary_element_fea.size();
    custom_vector<real3>& aabb_min_tet = data_manager->host_data.aabb_min_tet;
    custom_vector<real3>& aabb_max_tet = data_manager->host_data.aabb_max_tet;
    uint total_bins = (bins_per_axis.x + 1) * (bins_per_axis.y + 1) * (bins_per_axis.z + 1);
    is_rigid_bin_active.resize(total_bins);
    Thrust_Fill(is_rigid_bin_active, 1000000000);

#pragma omp parallel for
    for (int index = 0; index < (signed)f_number_of_bins_active; index++) {
        uint bin_number = f_bin_number_out[index];
        is_rigid_bin_active[bin_number] = index;
    }
    t_bin_intersections.resize(num_tets + 1);
    t_bin_intersections[num_tets] = 0;
#pragma omp parallel for
    for (int p = 0; p < num_tets; p++) {
        vec3 gmin = HashMin(aabb_min_tet[p], inv_bin_size);
        vec3 gmax = HashMax(aabb_max_tet[p], inv_bin_size);
        t_bin_intersections[p] = (gmax.x - gmin.x + 1) * (gmax.y - gmin.y + 1) * (gmax.z - gmin.z + 1);
    }
    Thrust_Exclusive_Scan(t_bin_intersections);
    uint t_number_of_bin_intersections = t_bin_intersections.back();

    t_bin_number.resize(t_number_of_bin_intersections);
    t_bin_number_out.resize(t_number_of_bin_intersections);
    t_bin_fluid_number.resize(t_number_of_bin_intersections);
    t_bin_start_index.resize(t_number_of_bin_intersections);

#pragma omp parallel for
    for (int p = 0; p < num_tets; p++) {
        uint count = 0, i, j, k;
        vec3 gmin = HashMin(aabb_min_tet[p], inv_bin_size);
        vec3 gmax = HashMax(aabb_max_tet[p], inv_bin_size);
        uint mInd = t_bin_intersections[p];
        for (i = (unsigned)gmin.x; i <= (unsigned)gmax.x; i++) {
            for (j = (unsigned)gmin.y; j <= (unsigned)gmax.y; j++) {
                for (k = (unsigned)gmin.z; k <= (unsigned)gmax.z; k++) {
                    t_bin_number[mInd + count] = Hash_Index(vec3(i, j, k), bins_per_axis);
                    t_bin_fluid_number[mInd + count] = p;
                    count++;
                }
            }
        }
    }
    Thrust_Sort_By_Key(t_bin_number, t_bin_fluid_number);
    uint t_number_of_bins_active = (int)(Run_Length_Encode(t_bin_number, t_bin_number_out, t_bin_start_index));

    t_bin_start_index.resize(t_number_of_bins_active + 1);
    t_bin_start_index[t_number_of_bins_active] = 0;
    Thrust_Exclusive_Scan(t_bin_start_index);
    custom_vector<uint> t_bin_num_contact(t_number_of_bins_active + 1);
    t_bin_num_contact[t_number_of_bins_active] = 0;

    norm_marker_tet.resize(num_tets * max_rigid_neighbors);
    cptb_marker_tet.resize(num_tets * max_rigid_neighbors);
    dpth_marker_tet.resize(num_tets * max_rigid_neighbors);
    face_marker_tet.resize(num_tets * max_rigid_neighbors);
    neighbor_marker_tet.resize(num_tets * max_rigid_neighbors);
    contact_counts.resize(num_tets + 1);
    short2 family = data_manager->fea_container->family;
    Thrust_Fill(contact_counts, 0);

    for (int index = 0; index < (signed)t_number_of_bins_active; index++) {
        uint bin_number = t_bin_number_out[index];
        unsigned int rigid_index = is_rigid_bin_active[bin_number];
        if (rigid_index != 1000000000) {
            uint start = t_bin_start_index[index];
            uint end = t_bin_start_index[index + 1];
            uint rigid_start = f_bin_start_index[rigid_index];
            uint rigid_end = f_bin_start_index[rigid_index + 1];
#pragma omp parallel for
            for (int i = start; i < (signed)end; i++) {
                uint p = t_bin_fluid_number[i];
                real3 Bmin = aabb_min_tet[p];
                real3 Bmax = aabb_max_tet[p];

                uvec4 tet_index = data_manager->host_data.tet_indices[data_manager->host_data.boundary_element_fea[p]];
                real3* node_pos = data_manager->host_data.pos_node_fea.data();
                uvec4 bface = data_manager->host_data.boundary_triangles_fea[p];
                real3 t1 = node_pos[bface.x];
                real3 t2 = node_pos[bface.y];
                real3 t3 = node_pos[bface.z];
                uint bf = bface.w;
                ConvexShapeTetradhedron* shapeB = new ConvexShapeTetradhedron(tet_index, node_pos);
                for (uint j = rigid_start; j < rigid_end; j++) {
                    uint fluid = f_bin_fluid_number[j];

                    real3 pos_sphere = pos_spheres[fluid];

                    real3 Amin = pos_sphere - real3(sphere_radius + collision_envelope) - global_origin;
                    real3 Amax = pos_sphere + real3(sphere_radius + collision_envelope) - global_origin;
                    if (current_bin(Amin, Amax, Bmin, Bmax, inv_bin_size, bins_per_axis, bin_number) == false) {
                        continue;
                    }
                    if (!overlap(Amin, Amax, Bmin, Bmax)) {
                        continue;
                    }
                    if (!collide(family, family_sphere))
                        continue;

                    ConvexShapeSphere* shapeA = new ConvexShapeSphere(pos_sphere, sphere_radius * .5);

                    real3 ptA, ptB, norm;
                    real depth;
                    real3 barycentric;
                    //int face;
                    real3 res;
                    if (MPRCollision(shapeA, shapeB, 0, norm, ptA, ptB, depth)) {
                        if (contact_counts[p] < max_rigid_neighbors) {
                            // FindTriIndex(ptB, tet_index, node_pos, face, barycentric);
                            // instead of finding the closest face, always use the surface face

                            SnapeToFaceBary(t1, t2, t3, ptB, res, barycentric);

                            norm_marker_tet[p * max_rigid_neighbors + contact_counts[p]] = norm;
                            cptb_marker_tet[p * max_rigid_neighbors + contact_counts[p]] = ptB;
                            dpth_marker_tet[p * max_rigid_neighbors + contact_counts[p]] = depth;
                            neighbor_marker_tet[p * max_rigid_neighbors + contact_counts[p]] = fluid;
                            face_marker_tet[p * max_rigid_neighbors + contact_counts[p]] = real4(barycentric, bf);
                            contact_counts[p]++;
                        }
                    }
                    delete shapeA;
                }
                delete shapeB;
            }
        }
    }

    // Compute a mapping with weights for each vertex in contact (sum up weights)

    Thrust_Exclusive_Scan(contact_counts);
    num_contacts = contact_counts[num_tets];
}
//==================================================================================================================================
}  // end namespace collision
}  // end namespace chrono
