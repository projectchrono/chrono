#include <algorithm>

#include "collision/ChCCollisionModel.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/collision/ChCNarrowphaseDispatch.h"
#include <chrono_parallel/collision/ChCNarrowphaseUtils.h>
#include "chrono_parallel/collision/ChCBroadphaseUtils.h"
#include "chrono_parallel/collision/ChCNarrowphaseMPR.h"
#include "chrono_parallel/collision/ChCNarrowphaseR.h"
#include "chrono_parallel/collision/ChCNarrowphaseGJK_EPA.h"

#include <thrust/remove.h>
#include <thrust/sort.h>
#include <thrust/transform_reduce.h>
#include <thrust/count.h>
#include <thrust/iterator/constant_iterator.h>

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
    if (data_manager->num_fea_nodes != 0) {
        DispatchRigidNode();
    }
}

void ChCNarrowphaseDispatch::PreprocessCount() {
    // MPR and GJK always report at most one contact per pair.
    if (narrowphase_algorithm == NARROWPHASE_MPR /*|| narrowphase_algorithm == NARROWPHASE_GJK*/) {
        thrust::fill(contact_index.begin(), contact_index.end(), 1);
        return;
    }

    // NarrowphaseR (and hence the hybrid algorithms) may produce different number
    // of contacts per pair, depending on the interacting shapes:
    //   - an interaction involving a sphere can produce at most one contact
    //   - an interaction involving a capsule can produce up to two contacts
    //   - a box-box interaction can produce up to 8 contacts

    // shape type (per shape)
    const shape_type* obj_data_T = data_manager->host_data.typ_rigid.data();
    // encoded shape IDs (per collision pair)
    const long long* collision_pair = data_manager->host_data.contact_pairs.data();

#pragma omp parallel for
    for (int index = 0; index < num_potential_rigid_contacts; index++) {
        // Identify the two candidate shapes and get their types.
        int2 pair = I2(int(collision_pair[index] >> 32), int(collision_pair[index] & 0xffffffff));
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
    LOG(TRACE) << "start PreprocessLocalToParent: ";
    uint num_shapes = data_manager->num_rigid_shapes;

    const custom_vector<int>& obj_data_T = data_manager->host_data.typ_rigid;
    const custom_vector<real3>& obj_data_A = data_manager->host_data.ObA_rigid;
    const custom_vector<real3>& obj_data_B = data_manager->host_data.ObB_rigid;
    const custom_vector<real3>& obj_data_C = data_manager->host_data.ObC_rigid;
    const custom_vector<quaternion>& obj_data_R = data_manager->host_data.ObR_rigid;
    const custom_vector<uint>& obj_data_ID = data_manager->host_data.id_rigid;

    const custom_vector<real3>& body_pos = data_manager->host_data.pos_rigid;
    const custom_vector<quaternion>& body_rot = data_manager->host_data.rot_rigid;

    data_manager->host_data.obj_data_A_global.resize(num_shapes);
    data_manager->host_data.obj_data_B_global.resize(num_shapes);
    data_manager->host_data.obj_data_C_global.resize(num_shapes);
    data_manager->host_data.obj_data_R_global.resize(num_shapes);

#pragma omp parallel for
    for (int index = 0; index < num_shapes; index++) {
        shape_type T = obj_data_T[index];

        // Get the identifier for the object associated with this collision shape
        uint ID = obj_data_ID[index];

        real3 pos = body_pos[ID];       // Get the global object position
        quaternion rot = body_rot[ID];  // Get the global object rotation

        data_manager->host_data.obj_data_A_global[index] = TransformLocalToParent(pos, rot, obj_data_A[index]);
        if (T == TRIANGLEMESH) {
            data_manager->host_data.obj_data_B_global[index] = TransformLocalToParent(pos, rot, obj_data_B[index]);
            data_manager->host_data.obj_data_C_global[index] = TransformLocalToParent(pos, rot, obj_data_C[index]);
        } else {
            data_manager->host_data.obj_data_B_global[index] = obj_data_B[index];
            data_manager->host_data.obj_data_C_global[index] = obj_data_C[index];
        }
        data_manager->host_data.obj_data_R_global[index] = Mult(rot, obj_data_R[index]);
    }
    LOG(TRACE) << "stop PreprocessLocalToParent: ";
}

void ChCNarrowphaseDispatch::Dispatch_Init(uint index,
                                           uint& icoll,
                                           uint& ID_A,
                                           uint& ID_B,
                                           ConvexShape& shapeA,
                                           ConvexShape& shapeB) {
    const shape_type* obj_data_T = data_manager->host_data.typ_rigid.data();
    const custom_vector<uint>& obj_data_ID = data_manager->host_data.id_rigid;
    const custom_vector<long long>& contact_pair = data_manager->host_data.contact_pairs;
    real3* convex_data = data_manager->host_data.convex_data.data();

    long long p = contact_pair[index];
    int2 pair =
        I2(int(p >> 32), int(p & 0xffffffff));  // Get the identifiers for the two shapes involved in this collision

    ID_A = obj_data_ID[pair.x];
    ID_B = obj_data_ID[pair.y];  // Get the identifiers of the two associated objects (bodies)

    shapeA.type = obj_data_T[pair.x];
    shapeB.type = obj_data_T[pair.y];  // Load the type data for each object in the collision pair

    shapeA.A = data_manager->host_data.obj_data_A_global[pair.x];
    shapeB.A = data_manager->host_data.obj_data_A_global[pair.y];
    shapeA.B = data_manager->host_data.obj_data_B_global[pair.x];
    shapeB.B = data_manager->host_data.obj_data_B_global[pair.y];
    shapeA.C = data_manager->host_data.obj_data_C_global[pair.x];
    shapeB.C = data_manager->host_data.obj_data_C_global[pair.y];
    shapeA.R = data_manager->host_data.obj_data_R_global[pair.x];
    shapeB.R = data_manager->host_data.obj_data_R_global[pair.y];
    shapeA.convex = convex_data;
    shapeB.convex = convex_data;

    //// TODO: what is the best way to dispatch this?
    icoll = contact_index[index];
}

void ChCNarrowphaseDispatch::Dispatch_Finalize(uint icoll, uint ID_A, uint ID_B, int nC) {
    custom_vector<int2>& body_ids = data_manager->host_data.bids_rigid_rigid;

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

#pragma omp parallel for
    for (int index = 0; index < num_potential_rigid_contacts; index++) {
        uint ID_A, ID_B, icoll;
        ConvexShape shapeA, shapeB;

        Dispatch_Init(index, icoll, ID_A, ID_B, shapeA, shapeB);

        if (MPRCollision(shapeA, shapeB, collision_envelope, norm[icoll], ptA[icoll], ptB[icoll],
                         contactDepth[icoll])) {
            effective_radius[icoll] = edge_radius;
            // The number of contacts reported by MPR is always 1.
            Dispatch_Finalize(icoll, ID_A, ID_B, 1);
        }
    }
}

void ChCNarrowphaseDispatch::DispatchGJK() {
    custom_vector<real3>& norm = data_manager->host_data.norm_rigid_rigid;
    custom_vector<real3>& ptA = data_manager->host_data.cpta_rigid_rigid;
    custom_vector<real3>& ptB = data_manager->host_data.cptb_rigid_rigid;
    custom_vector<real>& contactDepth = data_manager->host_data.dpth_rigid_rigid;
    custom_vector<real>& effective_radius = data_manager->host_data.erad_rigid_rigid;

#pragma omp parallel for
    for (int index = 0; index < num_potential_rigid_contacts; index++) {
        uint ID_A, ID_B, icoll;
        ConvexShape shapeA, shapeB;

        Dispatch_Init(index, icoll, ID_A, ID_B, shapeA, shapeB);

        ContactPoint contact_point;
        real3 separating_axis;
        if (GJKCollide(shapeA, shapeB, collision_envelope, contact_point, separating_axis)) {
            norm[icoll] = -contact_point.normal;
            ptA[icoll] = contact_point.pointA;
            ptB[icoll] = contact_point.pointB;
            contactDepth[icoll] = contact_point.depth;

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

#pragma omp parallel for
    for (int index = 0; index < num_potential_rigid_contacts; index++) {
        uint ID_A, ID_B, icoll;
        ConvexShape shapeA, shapeB;
        int nC;

        Dispatch_Init(index, icoll, ID_A, ID_B, shapeA, shapeB);

        if (RCollision(shapeA, shapeB, 2 * collision_envelope, &norm[icoll], &ptA[icoll], &ptB[icoll],
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

#pragma omp parallel for
    for (int index = 0; index < num_potential_rigid_contacts; index++) {
        uint ID_A, ID_B, icoll;
        ConvexShape shapeA, shapeB;
        int nC;

        Dispatch_Init(index, icoll, ID_A, ID_B, shapeA, shapeB);

        if (RCollision(shapeA, shapeB, 2 * collision_envelope, &norm[icoll], &ptA[icoll], &ptB[icoll],
                       &contactDepth[icoll], &effective_radius[icoll], nC)) {
            Dispatch_Finalize(icoll, ID_A, ID_B, nC);
        } else if (MPRCollision(shapeA, shapeB, collision_envelope, norm[icoll], ptA[icoll], ptB[icoll],
                                contactDepth[icoll])) {
            effective_radius[icoll] = edge_radius;
            Dispatch_Finalize(icoll, ID_A, ID_B, 1);
        }
    }
}

void ChCNarrowphaseDispatch::DispatchHybridGJK() {
    real3* norm = data_manager->host_data.norm_rigid_rigid.data();
    real3* ptA = data_manager->host_data.cpta_rigid_rigid.data();
    real3* ptB = data_manager->host_data.cptb_rigid_rigid.data();
    real* contactDepth = data_manager->host_data.dpth_rigid_rigid.data();
    real* effective_radius = data_manager->host_data.erad_rigid_rigid.data();

#pragma omp parallel for
    for (int index = 0; index < num_potential_rigid_contacts; index++) {
        uint ID_A, ID_B, icoll;
        ConvexShape shapeA, shapeB;
        int nC;

        Dispatch_Init(index, icoll, ID_A, ID_B, shapeA, shapeB);
        ContactPoint contact_point;
        real3 separating_axis;
        if (RCollision(shapeA, shapeB, 2 * collision_envelope, &norm[icoll], &ptA[icoll], &ptB[icoll],
                       &contactDepth[icoll], &effective_radius[icoll], nC)) {
            Dispatch_Finalize(icoll, ID_A, ID_B, nC);
        } else if (GJKCollide(shapeA, shapeB, collision_envelope, contact_point, separating_axis)) {
            norm[icoll] = -contact_point.normal;
            ptA[icoll] = contact_point.pointA;
            ptB[icoll] = contact_point.pointB;
            contactDepth[icoll] = contact_point.depth;

            effective_radius[icoll] = edge_radius;
            Dispatch_Finalize(icoll, ID_A, ID_B, 1);
        }
    }
}

void ChCNarrowphaseDispatch::DispatchRigid() {
    LOG(TRACE) << "start DispatchRigid: ";
    custom_vector<real3>& norm_data = data_manager->host_data.norm_rigid_rigid;
    custom_vector<real3>& cpta_data = data_manager->host_data.cpta_rigid_rigid;
    custom_vector<real3>& cptb_data = data_manager->host_data.cptb_rigid_rigid;
    custom_vector<real>& dpth_data = data_manager->host_data.dpth_rigid_rigid;
    custom_vector<real>& erad_data = data_manager->host_data.erad_rigid_rigid;
    custom_vector<int2>& bids_data = data_manager->host_data.bids_rigid_rigid;
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
        case NARROWPHASE_MPR:
            DispatchMPR();
            break;
        case NARROWPHASE_GJK:
            DispatchGJK();
            break;
        case NARROWPHASE_R:
            DispatchR();
            break;
        case NARROWPHASE_HYBRID_MPR:
            DispatchHybridMPR();
            break;
        case NARROWPHASE_HYBRID_GJK:
            DispatchHybridGJK();
            break;
    }

    num_rigid_contacts = Thrust_Count(contact_rigid_active, 1);
    // Remove elements corresponding to inactive contacts. We do this in one step,
    // using zip iterators and removing all entries for which contact_active is 'false'.
    thrust::remove_if(
        thrust::make_zip_iterator(thrust::make_tuple(norm_data.begin(), cpta_data.begin(), cptb_data.begin(),
                                                     dpth_data.begin(), erad_data.begin(), bids_data.begin())),
        thrust::make_zip_iterator(thrust::make_tuple(norm_data.end(), cpta_data.end(), cptb_data.end(), dpth_data.end(),
                                                     erad_data.end(), bids_data.end())),
        contact_rigid_active.begin(), thrust::logical_not<bool>());

    // Resize all lists so that we don't access invalid contacts
    norm_data.resize(num_rigid_contacts);
    cpta_data.resize(num_rigid_contacts);
    cptb_data.resize(num_rigid_contacts);
    dpth_data.resize(num_rigid_contacts);
    erad_data.resize(num_rigid_contacts);
    bids_data.resize(num_rigid_contacts);
    LOG(TRACE) << "stop DispatchRigid: ";
}

void ChCNarrowphaseDispatch::DispatchRigidFluid() {
    LOG(TRACE) << "start DispatchRigidFluid: ";

    real fluid_radius = data_manager->node_container->kernel_radius;
    int num_fluid_bodies = data_manager->num_fluid_bodies;
    int num_rigid_shapes = data_manager->num_rigid_shapes;
    real3 global_origin = data_manager->measures.collision.global_origin;
    int3 bins_per_axis = data_manager->settings.collision.bins_per_axis;
    real3 inv_bin_size = data_manager->measures.collision.inv_bin_size;
    custom_vector<real3>& sorted_pos_fluid = data_manager->host_data.sorted_pos_3dof;
    custom_vector<real3>& norm_rigid_fluid = data_manager->host_data.norm_rigid_fluid;
    custom_vector<real3>& cpta_rigid_fluid = data_manager->host_data.cpta_rigid_fluid;
    custom_vector<real>& dpth_rigid_fluid = data_manager->host_data.dpth_rigid_fluid;
    custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
    custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

    uint total_bins = (bins_per_axis.x + 1) * (bins_per_axis.y + 1) * (bins_per_axis.z + 1);
    is_rigid_bin_active.resize(total_bins);
    Thrust_Fill(is_rigid_bin_active, 1000000000);
#pragma omp parallel for
    for (int index = 0; index < data_manager->measures.collision.number_of_bins_active; index++) {
        uint bin_number = data_manager->host_data.bin_number_out[index];
        is_rigid_bin_active[bin_number] = index;
    }
    f_bin_intersections.resize(num_fluid_bodies + 1);
    f_bin_intersections[num_fluid_bodies] = 0;
#pragma omp parallel for
    for (int p = 0; p < num_fluid_bodies; p++) {
        real3 pos_fluid = sorted_pos_fluid[p];
        int3 gmin = HashMin(pos_fluid - real3(fluid_radius) - global_origin, inv_bin_size);
        int3 gmax = HashMax(pos_fluid + real3(fluid_radius) - global_origin, inv_bin_size);
        f_bin_intersections[p] = (gmax.x - gmin.x + 1) * (gmax.y - gmin.y + 1) * (gmax.z - gmin.z + 1);
    }
    Thrust_Exclusive_Scan(f_bin_intersections);
    uint f_number_of_bin_intersections = f_bin_intersections.back();

    f_bin_number.resize(f_number_of_bin_intersections);
    f_bin_number_out.resize(f_number_of_bin_intersections);
    f_bin_fluid_number.resize(f_number_of_bin_intersections);
    f_bin_start_index.resize(f_number_of_bin_intersections);

#pragma omp parallel for
    for (int p = 0; p < num_fluid_bodies; p++) {
        uint count = 0, i, j, k;
        real3 pos_fluid = sorted_pos_fluid[p];
        int3 gmin = HashMin(pos_fluid - real3(fluid_radius) - global_origin, inv_bin_size);
        int3 gmax = HashMax(pos_fluid + real3(fluid_radius) - global_origin, inv_bin_size);
        uint mInd = f_bin_intersections[p];
        for (i = gmin.x; i <= gmax.x; i++) {
            for (j = gmin.y; j <= gmax.y; j++) {
                for (k = gmin.z; k <= gmax.z; k++) {
                    f_bin_number[mInd + count] = Hash_Index(int3(i, j, k), bins_per_axis);
                    f_bin_fluid_number[mInd + count] = p;
                    count++;
                }
            }
        }
    }
    Thrust_Sort_By_Key(f_bin_number, f_bin_fluid_number);
    uint f_number_of_bins_active = Run_Length_Encode(f_bin_number, f_bin_number_out, f_bin_start_index);

    f_bin_start_index.resize(f_number_of_bins_active + 1);
    f_bin_start_index[f_number_of_bins_active] = 0;
    Thrust_Exclusive_Scan(f_bin_start_index);
    custom_vector<uint> f_bin_num_contact(f_number_of_bins_active + 1);
    f_bin_num_contact[f_number_of_bins_active] = 0;

    norm_rigid_fluid.resize(num_fluid_bodies * max_rigid_neighbors);
    cpta_rigid_fluid.resize(num_fluid_bodies * max_rigid_neighbors);
    dpth_rigid_fluid.resize(num_fluid_bodies * max_rigid_neighbors);
    neighbor_rigid_fluid.resize(num_fluid_bodies * max_rigid_neighbors);
    contact_counts.resize(num_fluid_bodies);

    Thrust_Fill(contact_counts, 0);
    for (int index = 0; index < f_number_of_bins_active; index++) {
        uint start = f_bin_start_index[index];
        uint end = f_bin_start_index[index + 1];
        uint count = 0;
        // Terminate early if there is only one object in the bin
        if (end - start == 1) {
            continue;
        }
        unsigned int rigid_index = is_rigid_bin_active[f_bin_number_out[index]];
        bool rigid_is_active = rigid_index != 1000000000;
        if (rigid_is_active) {
            uint rigid_start = data_manager->host_data.bin_start_index[rigid_index];
            uint rigid_end = data_manager->host_data.bin_start_index[rigid_index + 1];
#pragma omp parallel for
            for (uint i = start; i < end; i++) {
                uint p = f_bin_fluid_number[i];
                real3 pos_fluid = sorted_pos_fluid[p];
                real3 Bmin = pos_fluid - real3(fluid_radius) - global_origin;
                real3 Bmax = pos_fluid + real3(fluid_radius) - global_origin;

                for (uint j = rigid_start; j < rigid_end; j++) {
                    uint shape_id_a = data_manager->host_data.bin_aabb_number[j];
                    real3 Amin = data_manager->host_data.aabb_min[shape_id_a];
                    real3 Amax = data_manager->host_data.aabb_max[shape_id_a];
                    uint bodyA = data_manager->host_data.id_rigid[shape_id_a];
                    if (!overlap(Amin, Amax, Bmin, Bmax)) {
                        continue;
                    }

                    ConvexShape shapeA(data_manager->host_data.typ_rigid[shape_id_a],          //
                                       data_manager->host_data.obj_data_A_global[shape_id_a],  //
                                       data_manager->host_data.obj_data_B_global[shape_id_a],  //
                                       data_manager->host_data.obj_data_C_global[shape_id_a],  //
                                       data_manager->host_data.obj_data_R_global[shape_id_a],  //
                                       data_manager->host_data.convex_data.data());            //

                    ConvexShape shapeB(SPHERE, pos_fluid,                            //
                                       real3(fluid_radius, 0, 0),                    //
                                       real3(0),                                     //
                                       quaternion(1, 0, 0, 0),                       //
                                       data_manager->host_data.convex_data.data());  //

                    real3 ptA, ptB, norm;
                    real depth;

                    if (MPRCollision(shapeA, shapeB, collision_envelope, norm, ptA, ptB, depth)) {
                        if (contact_counts[p] < max_rigid_neighbors) {
                            norm_rigid_fluid[p * max_rigid_neighbors + contact_counts[p]] = norm;
                            cpta_rigid_fluid[p * max_rigid_neighbors + contact_counts[p]] = ptA;
                            dpth_rigid_fluid[p * max_rigid_neighbors + contact_counts[p]] = depth;
                            neighbor_rigid_fluid[p * max_rigid_neighbors + contact_counts[p]] = bodyA;
                            contact_counts[p]++;
                        }
                    }
                }
            }
        }
    }
    data_manager->num_rigid_fluid_contacts = Thrust_Total(contact_counts);
    LOG(TRACE) << "stop DispatchRigidFluid: " << data_manager->num_rigid_fluid_contacts;
}

inline int GridCoord(real x, real inv_bin_edge, real minimum) {
    real l = x - minimum;
    int c = round(l * inv_bin_edge);
    return c;
}

inline int GridHash(int x, int y, int z, const int3& bins_per_axis) {
    return ((z * bins_per_axis.y) * bins_per_axis.x) + (y * bins_per_axis.x) + x;
}

void ChCNarrowphaseDispatch::DispatchFluid() {
    LOG(TRACE) << "start DispatchFluidFluid: ";
    const int num_fluid_bodies = data_manager->num_fluid_bodies;
    const int num_rigid_bodies = data_manager->num_rigid_bodies;
    const int num_shafts = data_manager->num_shafts;

    if (num_fluid_bodies == 0) {
        return;
    }
    //=======
    const custom_vector<real3>& pos_fluid = data_manager->host_data.pos_3dof;
    const custom_vector<real3>& vel_fluid = data_manager->host_data.vel_3dof;
    custom_vector<real3>& sorted_pos_fluid = data_manager->host_data.sorted_pos_3dof;
    custom_vector<real3>& sorted_vel_fluid = data_manager->host_data.sorted_vel_3dof;

    custom_vector<int>& neighbor_fluid_fluid = data_manager->host_data.neighbor_3dof_3dof;
    custom_vector<int>& contact_counts = data_manager->host_data.c_counts_3dof_3dof;
    custom_vector<int>& particle_indices = data_manager->host_data.particle_indices_3dof;
    //custom_vector<int>& reverse_mapping = data_manager->host_data.reverse_mapping_3dof;

    neighbor_fluid_fluid.resize(num_fluid_bodies * max_neighbors);
    contact_counts.resize(num_fluid_bodies);
    particle_indices.resize(num_fluid_bodies);
    //reverse_mapping.resize(num_fluid_bodies);
    ff_bin_ids.resize(num_fluid_bodies);

    sorted_pos_fluid.resize(num_fluid_bodies);
    sorted_vel_fluid.resize(num_fluid_bodies);

    Thrust_Fill(contact_counts, 0);
    Thrust_Fill(neighbor_fluid_fluid, 0);

    const real radius = data_manager->node_container->kernel_radius;
    const real radiusSq = radius * radius;

    bbox res(pos_fluid[0], pos_fluid[0]);
    bbox_transformation unary_op;
    bbox_reduction binary_op;
    res = thrust::transform_reduce(pos_fluid.begin(), pos_fluid.end(), unary_op, res, binary_op);

    real3& max_bounding_point = data_manager->measures.collision.ff_max_bounding_point;
    real3& min_bounding_point = data_manager->measures.collision.ff_min_bounding_point;
    int3& bins_per_axis = data_manager->measures.collision.ff_bins_per_axis;

    max_bounding_point = real3(Max(Ceil(res.second.x), Ceil(res.second.x + radius * 6)),
                               Max(Ceil(res.second.y), Ceil(res.second.y + radius * 6)),
                               Max(Ceil(res.second.z), Ceil(res.second.z + radius * 6)));

    min_bounding_point = real3(Min(Floor(res.first.x), Floor(res.first.x - radius * 6)),
                               Min(Floor(res.first.y), Floor(res.first.y - radius * 6)),
                               Min(Floor(res.first.z), Floor(res.first.z - radius * 6)));

    real3 diag = max_bounding_point - min_bounding_point;

    bins_per_axis = int3(diag / (radius * 2));

    real inv_bin_edge = real(1.0) / (radius * 2 + data_manager->node_container->collision_envelope);
    size_t grid_size = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;

    ff_bin_starts.resize(grid_size);
    ff_bin_ends.resize(grid_size);

    Thrust_Fill(ff_bin_starts, 0);
    Thrust_Fill(ff_bin_ends, 0);
    Thrust_Fill(contact_counts, 0);

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
    for (int i = 0; i < num_fluid_bodies; i++) {
        int index = particle_indices[i];
        sorted_pos_fluid[i] = pos_fluid[index];
        sorted_vel_fluid[i] = vel_fluid[index];
        data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 0] = vel_fluid[index].x;
        data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 1] = vel_fluid[index].y;
        data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 2] = vel_fluid[index].z;

        //reverse_mapping[index] = i;
    }

#pragma omp parallel for
    for (int p = 0; p < num_fluid_bodies; p++) {
        real3 xi = sorted_pos_fluid[p];
        const int cx = GridCoord(xi.x, inv_bin_edge, min_bounding_point.x);
        const int cy = GridCoord(xi.y, inv_bin_edge, min_bounding_point.y);
        const int cz = GridCoord(xi.z, inv_bin_edge, min_bounding_point.z);

        const int xstart = cx - 1, xend = cx + 1;
        const int ystart = cy - 1, yend = cy + 1;
        const int zstart = cz - 1, zend = cz + 1;

        int contact_count = 0;
        for (int k = zstart; k <= zend; ++k) {
            for (int j = ystart; j <= yend; ++j) {
                for (int i = xstart; i <= xend; ++i) {
                    const int cellIndex = GridHash(i, j, k, bins_per_axis);
                    const int cellStart = ff_bin_starts[cellIndex];
                    const int cellEnd = ff_bin_ends[cellIndex];
                    for (int q = cellStart; q < cellEnd; ++q) {
                        // if (q == p) {
                        //    continue;
                        //}  // disabled this so that we get self contact
                        const real3 xj = sorted_pos_fluid[q];
                        const real3 xij = xi - xj;
                        const real dSq = Dot(xij);
                        if (dSq < radiusSq) {
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

    data_manager->num_fluid_contacts = Thrust_Total(contact_counts);
    LOG(TRACE) << "stop DispatchFluidFluid: " << data_manager->num_fluid_contacts;
}

void ChCNarrowphaseDispatch::DispatchRigidNode() {
    LOG(TRACE) << "start DispatchRigidNode: ";

    real node_radius = data_manager->fea_container->kernel_radius;
    int num_nodes = data_manager->num_fea_nodes;
    int num_rigid_shapes = data_manager->num_rigid_shapes;

    real3 global_origin = data_manager->measures.collision.global_origin;
    int3 bins_per_axis = data_manager->settings.collision.bins_per_axis;
    real3 inv_bin_size = data_manager->measures.collision.inv_bin_size;

    // printf("GRID %f %f %f [%d %d %d] [%f %f %f] nodes: %d\n", inv_bin_size.x, inv_bin_size.y, inv_bin_size.z,
    // bins_per_axis.x,
    //       bins_per_axis.y, bins_per_axis.z, global_origin.x, global_origin.y, global_origin.z, num_nodes);

    custom_vector<real3>& pos_node = data_manager->host_data.pos_node;

    custom_vector<real3>& norm_rigid_node = data_manager->host_data.norm_rigid_node;
    custom_vector<real3>& cpta_rigid_node = data_manager->host_data.cpta_rigid_node;
    custom_vector<real>& dpth_rigid_node = data_manager->host_data.dpth_rigid_node;
    custom_vector<int>& neighbor_rigid_node = data_manager->host_data.neighbor_rigid_node;
    custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_node;
    custom_vector<real3>& pos_nodes = data_manager->host_data.pos_node;
    uint total_bins = (bins_per_axis.x + 1) * (bins_per_axis.y + 1) * (bins_per_axis.z + 1);

    is_rigid_bin_active.resize(total_bins);
    Thrust_Fill(is_rigid_bin_active, 1000000000);
#pragma omp parallel for
    for (int index = 0; index < data_manager->measures.collision.number_of_bins_active; index++) {
        uint bin_number = data_manager->host_data.bin_number_out[index];
        is_rigid_bin_active[bin_number] = index;
    }
    n_bin_intersections.resize(num_nodes + 1);
    n_bin_intersections[num_nodes] = 0;
#pragma omp parallel for
    for (int p = 0; p < num_nodes; p++) {
        real3 pos_node = pos_nodes[p];
        int3 gmin = HashMin(pos_node - real3(node_radius) - global_origin, inv_bin_size);
        int3 gmax = HashMax(pos_node + real3(node_radius) - global_origin, inv_bin_size);
        n_bin_intersections[p] = (gmax.x - gmin.x + 1) * (gmax.y - gmin.y + 1) * (gmax.z - gmin.z + 1);
    }
    Thrust_Exclusive_Scan(n_bin_intersections);
    uint n_number_of_bin_intersections = n_bin_intersections.back();

    n_bin_number.resize(n_number_of_bin_intersections);
    n_bin_number_out.resize(n_number_of_bin_intersections);
    n_bin_node_number.resize(n_number_of_bin_intersections);
    n_bin_start_index.resize(n_number_of_bin_intersections);

#pragma omp parallel for
    for (int p = 0; p < num_nodes; p++) {
        uint count = 0, i, j, k;
        real3 pos_node = pos_nodes[p];
        int3 gmin = HashMin(pos_node - real3(node_radius) - global_origin, inv_bin_size);
        int3 gmax = HashMax(pos_node + real3(node_radius) - global_origin, inv_bin_size);
        uint mInd = n_bin_intersections[p];
        for (i = gmin.x; i <= gmax.x; i++) {
            for (j = gmin.y; j <= gmax.y; j++) {
                for (k = gmin.z; k <= gmax.z; k++) {
                    n_bin_number[mInd + count] = Hash_Index(int3(i, j, k), bins_per_axis);
                    n_bin_node_number[mInd + count] = p;

                    // printf("n_bin_number:  %d, n_bin_node_number %d \n", n_bin_number[mInd + count],
                    // n_bin_node_number[mInd + count]);

                    count++;
                }
            }
        }
    }
    Thrust_Sort_By_Key(n_bin_number, n_bin_node_number);
    uint n_number_of_bins_active = Run_Length_Encode(n_bin_number, n_bin_number_out, n_bin_start_index);

    n_bin_start_index.resize(n_number_of_bins_active + 1);
    n_bin_start_index[n_number_of_bins_active] = 0;
    Thrust_Exclusive_Scan(n_bin_start_index);
    custom_vector<uint> n_bin_num_contact(n_number_of_bins_active + 1);
    n_bin_num_contact[n_number_of_bins_active] = 0;

    norm_rigid_node.resize(num_nodes * max_rigid_neighbors);
    cpta_rigid_node.resize(num_nodes * max_rigid_neighbors);
    dpth_rigid_node.resize(num_nodes * max_rigid_neighbors);
    neighbor_rigid_node.resize(num_nodes * max_rigid_neighbors);
    contact_counts.resize(num_nodes);

    Thrust_Fill(contact_counts, 0);
    for (int index = 0; index < n_number_of_bins_active; index++) {
        uint start = n_bin_start_index[index];
        uint end = n_bin_start_index[index + 1];
        uint count = 0;
        // Terminate early if there is only one object in the bin
        if (end - start == 1) {
            continue;
        }

        // printf("n_bin_number_out: %d %d\n",index, n_bin_number_out[index]);
        unsigned int rigid_index = is_rigid_bin_active[n_bin_number_out[index]];
        bool rigid_is_active = rigid_index != 1000000000;
        if (rigid_is_active) {
            uint rigid_start = data_manager->host_data.bin_start_index[rigid_index];
            uint rigid_end = data_manager->host_data.bin_start_index[rigid_index + 1];
#pragma omp parallel for
            for (uint i = start; i < end; i++) {
                uint p = n_bin_node_number[i];
                real3 pos_node = pos_nodes[p];
                real3 Bmin = pos_node - real3(node_radius) - global_origin;
                real3 Bmax = pos_node + real3(node_radius) - global_origin;

                for (uint j = rigid_start; j < rigid_end; j++) {
                    uint shape_id_a = data_manager->host_data.bin_aabb_number[j];
                    real3 Amin = data_manager->host_data.aabb_min[shape_id_a];
                    real3 Amax = data_manager->host_data.aabb_max[shape_id_a];
                    uint bodyA = data_manager->host_data.id_rigid[shape_id_a];
                    if (!overlap(Amin, Amax, Bmin, Bmax)) {
                        continue;
                    }

                    ConvexShape shapeA(data_manager->host_data.typ_rigid[shape_id_a],          //
                                       data_manager->host_data.obj_data_A_global[shape_id_a],  //
                                       data_manager->host_data.obj_data_B_global[shape_id_a],  //
                                       data_manager->host_data.obj_data_C_global[shape_id_a],  //
                                       data_manager->host_data.obj_data_R_global[shape_id_a],  //
                                       data_manager->host_data.convex_data.data());            //

                    ConvexShape shapeB(SPHERE, pos_node,                             //
                                       real3(node_radius, 0, 0),                     //
                                       real3(0),                                     //
                                       quaternion(1, 0, 0, 0),                       //
                                       data_manager->host_data.convex_data.data());  //

                    real3 ptA, ptB, norm;
                    real depth;

                    if (MPRCollision(shapeA, shapeB, collision_envelope, norm, ptA, ptB, depth)) {
                        if (contact_counts[p] < max_rigid_neighbors) {
                            norm_rigid_node[p * max_rigid_neighbors + contact_counts[p]] = norm;
                            cpta_rigid_node[p * max_rigid_neighbors + contact_counts[p]] = ptA;
                            dpth_rigid_node[p * max_rigid_neighbors + contact_counts[p]] = depth;
                            neighbor_rigid_node[p * max_rigid_neighbors + contact_counts[p]] = bodyA;
                            contact_counts[p]++;
                        }
                    }
                }
            }
        }
    }
    data_manager->num_rigid_node_contacts = Thrust_Total(contact_counts);
    LOG(TRACE) << "stop DispatchRigidNode: " << data_manager->num_rigid_node_contacts;
}

}  // end namespace collision
}  // end namespace chrono
