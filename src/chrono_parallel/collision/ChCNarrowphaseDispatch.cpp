#include <algorithm>

#include "collision/ChCCollisionModel.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/collision/ChCNarrowphaseDispatch.h"
#include <chrono_parallel/collision/ChCNarrowphaseUtils.h>
#include "chrono_parallel/collision/ChCNarrowphaseMPR.h"
#include "chrono_parallel/collision/ChCNarrowphaseR.h"
#include "chrono_parallel/collision/ChCNarrowphaseGJK_EPA.h"
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

    //  if (num_potential_rigid_fluid_contacts == 0) {
    //    data_manager->host_data.norm_rigid_fluid.resize(0);
    //    data_manager->host_data.cpta_rigid_fluid.resize(0);
    //    data_manager->host_data.dpth_rigid_fluid.resize(0);
    //    data_manager->host_data.bids_rigid_fluid.resize(0);
    //  }
    //  if (num_potential_fluid_contacts == 0) {
    //    data_manager->host_data.bids_fluid_fluid.resize(0);
    //  }
}

void ChCNarrowphaseDispatch::Process() {
    //======== Indexing variables and other information
    num_potential_rigid_contacts = data_manager->num_rigid_contacts;
    num_potential_rigid_fluid_contacts = data_manager->num_rigid_fluid_contacts;
    num_potential_fluid_contacts = data_manager->num_fluid_contacts;
    narrowphase_algorithm = data_manager->settings.collision.narrowphase_algorithm;
    collision_envelope = data_manager->settings.collision.collision_envelope;
    ClearContacts();
    // Transform Rigid body shapes to global coordinate system
    PreprocessLocalToParent();
    LOG(TRACE) << "PreprocessLocalToParent: ";
    if (num_potential_rigid_contacts != 0) {
        DispatchRigid();
    }
    LOG(TRACE) << "DispatchRigid: ";
    if (data_manager->num_fluid_bodies != 0) {
        DispatchFluid();
        LOG(TRACE) << "DispatchFluid: ";
        DispatchRigidFluid();
        LOG(TRACE) << "DispatchRigidFluid: ";
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
    uint num_shapes = data_manager->num_rigid_shapes;

    const host_vector<int>& obj_data_T = data_manager->host_data.typ_rigid;
    const host_vector<real3>& obj_data_A = data_manager->host_data.ObA_rigid;
    const host_vector<real3>& obj_data_B = data_manager->host_data.ObB_rigid;
    const host_vector<real3>& obj_data_C = data_manager->host_data.ObC_rigid;
    const host_vector<real4>& obj_data_R = data_manager->host_data.ObR_rigid;
    const host_vector<uint>& obj_data_ID = data_manager->host_data.id_rigid;

    const host_vector<real3>& body_pos = data_manager->host_data.pos_rigid;
    const host_vector<real4>& body_rot = data_manager->host_data.rot_rigid;

    obj_data_A_global.resize(num_shapes);
    obj_data_B_global.resize(num_shapes);
    obj_data_C_global.resize(num_shapes);
    obj_data_R_global.resize(num_shapes);

#pragma omp parallel for
    for (int index = 0; index < num_shapes; index++) {
        shape_type T = obj_data_T[index];

        // Get the identifier for the object associated with this collision shape
        uint ID = obj_data_ID[index];

        real3 pos = body_pos[ID];  // Get the global object position
        real4 rot = body_rot[ID];  // Get the global object rotation

        obj_data_A_global[index] = TransformLocalToParent(pos, rot, obj_data_A[index]);
        if (T == TRIANGLEMESH) {
            obj_data_B_global[index] = TransformLocalToParent(pos, rot, obj_data_B[index]);
            obj_data_C_global[index] = TransformLocalToParent(pos, rot, obj_data_C[index]);
        } else {
            obj_data_B_global[index] = obj_data_B[index];
            obj_data_C_global[index] = obj_data_C[index];
        }
        obj_data_R_global[index] = mult(rot, obj_data_R[index]);
    }
}

void ChCNarrowphaseDispatch::Dispatch_Init(uint index,
                                           uint& icoll,
                                           uint& ID_A,
                                           uint& ID_B,
                                           ConvexShape& shapeA,
                                           ConvexShape& shapeB) {
    const shape_type* obj_data_T = data_manager->host_data.typ_rigid.data();
    const host_vector<uint>& obj_data_ID = data_manager->host_data.id_rigid;
    const host_vector<long long>& contact_pair = data_manager->host_data.contact_pairs;
    const host_vector<real>& collision_margins = data_manager->host_data.margin_rigid;
    real3* convex_data = data_manager->host_data.convex_data.data();

    long long p = contact_pair[index];
    int2 pair =
        I2(int(p >> 32), int(p & 0xffffffff));  // Get the identifiers for the two shapes involved in this collision

    ID_A = obj_data_ID[pair.x];
    ID_B = obj_data_ID[pair.y];  // Get the identifiers of the two associated objects (bodies)

    shapeA.type = obj_data_T[pair.x];
    shapeB.type = obj_data_T[pair.y];  // Load the type data for each object in the collision pair

    shapeA.A = obj_data_A_global[pair.x];
    shapeB.A = obj_data_A_global[pair.y];
    shapeA.B = obj_data_B_global[pair.x];
    shapeB.B = obj_data_B_global[pair.y];
    shapeA.C = obj_data_C_global[pair.x];
    shapeB.C = obj_data_C_global[pair.y];
    shapeA.R = obj_data_R_global[pair.x];
    shapeB.R = obj_data_R_global[pair.y];
    shapeA.convex = convex_data;
    shapeB.convex = convex_data;
    shapeA.margin = collision_margins[pair.x];
    shapeB.margin = collision_margins[pair.y];

    //// TODO: what is the best way to dispatch this?
    icoll = contact_index[index];
}

void ChCNarrowphaseDispatch::Dispatch_Finalize(uint icoll, uint ID_A, uint ID_B, int nC) {
    host_vector<int2>& body_ids = data_manager->host_data.bids_rigid_rigid;

    // Mark the active contacts and set their body IDs
    for (int i = 0; i < nC; i++) {
        contact_rigid_active[icoll + i] = true;
        body_ids[icoll + i] = I2(ID_A, ID_B);
    }
}

void ChCNarrowphaseDispatch::DispatchMPR() {
    host_vector<real3>& norm = data_manager->host_data.norm_rigid_rigid;
    host_vector<real3>& ptA = data_manager->host_data.cpta_rigid_rigid;
    host_vector<real3>& ptB = data_manager->host_data.cptb_rigid_rigid;
    host_vector<real>& contactDepth = data_manager->host_data.dpth_rigid_rigid;
    host_vector<real>& effective_radius = data_manager->host_data.erad_rigid_rigid;

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
    host_vector<real3>& norm = data_manager->host_data.norm_rigid_rigid;
    host_vector<real3>& ptA = data_manager->host_data.cpta_rigid_rigid;
    host_vector<real3>& ptB = data_manager->host_data.cptb_rigid_rigid;
    host_vector<real>& contactDepth = data_manager->host_data.dpth_rigid_rigid;
    host_vector<real>& effective_radius = data_manager->host_data.erad_rigid_rigid;

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
    host_vector<real3>& norm_data = data_manager->host_data.norm_rigid_rigid;
    host_vector<real3>& cpta_data = data_manager->host_data.cpta_rigid_rigid;
    host_vector<real3>& cptb_data = data_manager->host_data.cptb_rigid_rigid;
    host_vector<real>& dpth_data = data_manager->host_data.dpth_rigid_rigid;
    host_vector<real>& erad_data = data_manager->host_data.erad_rigid_rigid;
    host_vector<int2>& bids_data = data_manager->host_data.bids_rigid_rigid;
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
}

// Check if two AABBs overlap using their min/max corners.
inline bool overlap(real3 Amin, real3 Amax, real3 Bmin, real3 Bmax) {
    // Return true only if the two AABBs overlap in all 3 directions.
    return (Amin.x <= Bmax.x && Bmin.x <= Amax.x) && (Amin.y <= Bmax.y && Bmin.y <= Amax.y) &&
           (Amin.z <= Bmax.z && Bmin.z <= Amax.z);
}

void ChCNarrowphaseDispatch::DispatchRigidFluid() {
    LOG(TRACE) << "start DispatchRigidFluid: ";
#if 0
    data_manager->num_rigid_fluid_contacts = 0;
    data_manager->host_data.c_counts_rigid_fluid.resize(data_manager->num_fluid_bodies);
    Thrust_Fill(data_manager->host_data.c_counts_rigid_fluid, 0);
#else
    host_vector<real3>& pos_fluid = data_manager->host_data.pos_fluid;
    host_vector<real3>& norm_rigid_fluid = data_manager->host_data.norm_rigid_fluid;
    host_vector<real3>& cpta_rigid_fluid = data_manager->host_data.cpta_rigid_fluid;
    host_vector<real>& dpth_rigid_fluid = data_manager->host_data.dpth_rigid_fluid;
    host_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
    host_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

    real fluid_radius = data_manager->settings.fluid.kernel_radius;

    int num_fluid_bodies = data_manager->num_fluid_bodies;
    int num_rigid_shapes = data_manager->num_rigid_shapes;

    norm_rigid_fluid.resize(num_fluid_bodies * max_rigid_neighbors);
    cpta_rigid_fluid.resize(num_fluid_bodies * max_rigid_neighbors);
    dpth_rigid_fluid.resize(num_fluid_bodies * max_rigid_neighbors);
    neighbor_rigid_fluid.resize(num_fluid_bodies * max_rigid_neighbors);
    contact_counts.resize(num_fluid_bodies);

    real radius = data_manager->settings.fluid.kernel_radius;

    //#pragma omp parallel for
    for (int p = 0; p < num_fluid_bodies; p++) {
        int counter = 0;
        for (int r = 0; r < num_rigid_shapes; ++r) {
            //            if (overlap(data_manager->host_data.aabb_min[r], data_manager->host_data.aabb_max[r],
            //                        pos_fluid[p] - real3(radius), pos_fluid[p] + real3(radius)) == false) {
            //                continue;
            //            }

            ConvexShape shapeA, shapeB;
            shapeA.type =
                data_manager->host_data.typ_rigid[r];  // Load the type data for each object in the collision pair

            shapeB.type = SPHERE;

            shapeA.A = obj_data_A_global[r];
            shapeB.A = pos_fluid[p];

            shapeA.B = obj_data_B_global[r];
            shapeB.B = real3(radius, 0, 0);

            shapeA.C = obj_data_C_global[r];
            shapeB.C = real3(0);

            shapeA.R = obj_data_R_global[r];
            shapeB.R = quaternion(1, 0, 0, 0);

            shapeA.convex = data_manager->host_data.convex_data.data();
            shapeB.convex = data_manager->host_data.convex_data.data();

            shapeB.margin = 0;
            shapeA.margin = 0;

            real3 ptA, ptB, norm;
            real depth, erad;
            int nC = 0;
            //            if (RCollision(shapeA, shapeB, 2 * collision_envelope, &norm, &ptA, &ptB, &depth, &erad, nC))
            //            {
            //                if (counter < max_rigid_neighbors) {
            //                    norm_rigid_fluid[p * max_rigid_neighbors + counter] = norm;
            //                    cpta_rigid_fluid[p * max_rigid_neighbors + counter] = ptA;
            //                    dpth_rigid_fluid[p * max_rigid_neighbors + counter] = depth;
            //                    bids_rigid_fluid[p * max_rigid_neighbors + counter] =
            //                    data_manager->host_data.id_rigid[r];
            //                    ++counter;
            //                }
            //            } else
            if (MPRCollision(shapeA, shapeB, collision_envelope, norm, ptA, ptB, depth)) {
                if (counter < max_rigid_neighbors) {
                    norm_rigid_fluid[p * max_rigid_neighbors + counter] = norm;
                    cpta_rigid_fluid[p * max_rigid_neighbors + counter] = ptA;
                    dpth_rigid_fluid[p * max_rigid_neighbors + counter] = depth;
                    neighbor_rigid_fluid[p * max_rigid_neighbors + counter] = data_manager->host_data.id_rigid[r];
                    ++counter;
                }
            }
        }
        contact_counts[p] = counter;
    }

    data_manager->num_rigid_fluid_contacts = Thrust_Total(contact_counts);

//    std::cout << "FL_RIG " << data_manager->num_rigid_fluid_contacts << std::endl;
//    if (data_manager->num_rigid_fluid_contacts > 0) {
//        data_manager->host_data.bids_rigid_fluid.resize(data_manager->num_rigid_fluid_contacts);
//        data_manager->host_data.norm_rigid_fluid.resize(data_manager->num_rigid_fluid_contacts);
//        data_manager->host_data.cpta_rigid_fluid.resize(data_manager->num_rigid_fluid_contacts);
//        data_manager->host_data.dpth_rigid_fluid.resize(data_manager->num_rigid_fluid_contacts);
//        int counter = 0;
//        for (int p = 0; p < num_fluid_bodies; p++) {
//            for (int i = 0; i < contact_counts[p]; ++i) {
//                int r = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
//
//                data_manager->host_data.bids_rigid_fluid[counter] = I2(r, p);
//                data_manager->host_data.norm_rigid_fluid[counter] = norm_rigid_fluid[p * max_rigid_neighbors + i];
//                data_manager->host_data.cpta_rigid_fluid[counter] = cpta_rigid_fluid[p * max_rigid_neighbors + i];
//                data_manager->host_data.dpth_rigid_fluid[counter] = dpth_rigid_fluid[p * max_rigid_neighbors + i];
//                // std::cout << "FL_RIG: " << r << " " << p << std::endl;
//
//                counter++;
//            }
//        }
//    }
#endif
    //    for (int p = 0; p < num_fluid_bodies; p++) {
    //        if (contact_counts[p] > 0) {
    //            std::cout << "p: " << p << " [" << contact_counts[p] << "] ";
    //            for (int i = 0; i < contact_counts[p]; i++) {
    //                std::cout << bids_rigid_fluid[p * max_neighbors + i] << " ";
    //            }
    //            std::cout << std::endl;
    //        }
    //    }
}

inline int GridCoord(real x, real inv_bin_edge, real minimum) {
    real l = x - minimum;
    int c = round(l * inv_bin_edge);
    return c;
}

inline int GridHash(int x, int y, int z, const int3& bins_per_axis) {
    return ((z * bins_per_axis.y) * bins_per_axis.x) + (y * bins_per_axis.x) + x;
}

typedef thrust::pair<real3, real3> bbox;

// reduce a pair of bounding boxes (a,b) to a bounding box containing a and b
struct bbox_reduction : public thrust::binary_function<bbox, bbox, bbox> {
    bbox operator()(bbox a, bbox b) {
        real3 ll =
            R3(Min(a.first.x, b.first.x), Min(a.first.y, b.first.y), Min(a.first.z, b.first.z));  // lower left corner
        real3 ur = R3(Max(a.second.x, b.second.x), Max(a.second.y, b.second.y),
                      Max(a.second.z, b.second.z));  // upper right corner
        return bbox(ll, ur);
    }
};

// convert a point to a bbox containing that point, (point) -> (point, point)
struct bbox_transformation : public thrust::unary_function<real3, bbox> {
    bbox operator()(real3 point) { return bbox(point, point); }
};

void ChCNarrowphaseDispatch::DispatchFluid() {
    const int num_fluid_bodies = data_manager->num_fluid_bodies;
    std::cout << "FLUIDCOTNACT\n";
    if (num_fluid_bodies == 0) {
        return;
    }

    const host_vector<real3>& pos_fluid = data_manager->host_data.pos_fluid;
    host_vector<real3>& sorted_pos_fluid = data_manager->host_data.sorted_pos_fluid;

    host_vector<int>& neighbor_fluid_fluid = data_manager->host_data.neighbor_fluid_fluid;
    host_vector<int>& contact_counts = data_manager->host_data.c_counts_fluid_fluid;

    host_vector<int> bin_ids(num_fluid_bodies);
    host_vector<int>& particle_indices = data_manager->host_data.particle_indices_fluid;
    particle_indices.resize(num_fluid_bodies);

    host_vector<int> reverse_mapping(num_fluid_bodies);

    contact_counts.resize(num_fluid_bodies);
    sorted_pos_fluid.resize(num_fluid_bodies);
    neighbor_fluid_fluid.resize(num_fluid_bodies * max_neighbors);

    Thrust_Fill(contact_counts, 0);
    Thrust_Fill(neighbor_fluid_fluid, 0);

    const real radius = data_manager->settings.fluid.kernel_radius;
    const real radiusSq = radius * radius;

    bbox res(pos_fluid[0], pos_fluid[0]);
    bbox_transformation unary_op;
    bbox_reduction binary_op;
    res = thrust::transform_reduce(pos_fluid.begin(), pos_fluid.end(), unary_op, res, binary_op);

    real3 max_bounding_point = real3(std::max(std::ceil(res.second.x), std::ceil(res.second.x + radius * 6)),
                                     std::max(std::ceil(res.second.y), std::ceil(res.second.y + radius * 6)),
                                     std::max(std::ceil(res.second.z), std::ceil(res.second.z + radius * 6)));

    real3 min_bounding_point = real3(std::min(std::floor(res.first.x), std::floor(res.first.x - radius * 6)),
                                     std::min(std::floor(res.first.y), std::floor(res.first.y - radius * 6)),
                                     std::min(std::floor(res.first.z), std::floor(res.first.z - radius * 6)));

    real3 diag = max_bounding_point - min_bounding_point;
    int3 bins_per_axis;

    bins_per_axis.x = (diag.x / (radius * 2));
    bins_per_axis.y = (diag.y / (radius * 2));
    bins_per_axis.z = (diag.z / (radius * 2));

    real inv_bin_edge = 1.f / (radius * 2 + data_manager->settings.fluid.collision_envelope);
    size_t grid_size = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;

    host_vector<int> bin_starts(grid_size);
    host_vector<int> bin_ends(grid_size);

    Thrust_Fill(bin_starts, 0);
    Thrust_Fill(bin_ends, 0);
    Thrust_Fill(contact_counts, 0);

#pragma omp parallel for
    for (int i = 0; i < num_fluid_bodies; i++) {
        real3 p = pos_fluid[i];
        bin_ids[i] = GridHash(GridCoord(p.x, inv_bin_edge, min_bounding_point.x),
                              GridCoord(p.y, inv_bin_edge, min_bounding_point.y),
                              GridCoord(p.z, inv_bin_edge, min_bounding_point.z), bins_per_axis);
        particle_indices[i] = i;
    }

    Thrust_Sort_By_Key(bin_ids, particle_indices);

#pragma omp parallel for
    for (int i = 0; i < num_fluid_bodies; i++) {
        int c = bin_ids[i];
        if (i == 0) {
            bin_starts[c] = i;
        } else {
            int p = bin_ids[i - 1];
            if (c != p) {
                bin_starts[c] = i;
                bin_ends[p] = i;
            }
        }
        if (i == num_fluid_bodies - 1) {
            bin_ends[c] = i + 1;
        }
    }

#pragma omp parallel for
    for (int i = 0; i < num_fluid_bodies; i++) {
        int index = particle_indices[i];
        sorted_pos_fluid[i] = pos_fluid[index];
        reverse_mapping[index] = i;
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
                    const int cellStart = bin_starts[cellIndex];
                    const int cellEnd = bin_ends[cellIndex];
                    for (int q = cellStart; q < cellEnd; ++q) {
                        if (q == p) {
                            continue;
                        }  // disabled this so that we get self contact
                        const real3 xj = sorted_pos_fluid[q];
                        const real3 xij = xi - xj;
                        const real dSq = dot(xij, xij);
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
    uint& num_fluid_contacts = data_manager->num_fluid_contacts;
    num_fluid_contacts = Thrust_Total(contact_counts);

    //    for (int p = 0; p < num_fluid_bodies; p++) {
    //        std::cout << "p: " << p << " ";
    //        for (int i = 0; i < contact_counts[p]; i++) {
    //            std::cout << neighbor_fluid_fluid[p * max_neighbors + i] << " ";
    //        }
    //        std::cout << std::endl;
    //    }
    std::cout << "FLUID DONE\n";
}

}  // end namespace collision
}  // end namespace chrono
