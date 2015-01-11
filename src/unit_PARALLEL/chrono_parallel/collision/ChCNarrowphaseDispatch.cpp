#include <algorithm>

#include "collision/ChCCollisionModel.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/collision/ChCNarrowphaseDispatch.h"
#include "chrono_parallel/collision/ChCNarrowphaseMPRUtils.h"
#include "chrono_parallel/collision/ChCNarrowphaseMPR.h"
#include "chrono_parallel/collision/ChCNarrowphaseR.h"

namespace chrono {
namespace collision {

void ChCNarrowphaseDispatch::Process(ChParallelDataManager* data_container) {
  //======== Collision output data for rigid contacts
  custom_vector<real3>& norm_data = data_container->host_data.norm_rigid_rigid;
  custom_vector<real3>& cpta_data = data_container->host_data.cpta_rigid_rigid;
  custom_vector<real3>& cptb_data = data_container->host_data.cptb_rigid_rigid;
  custom_vector<real>& dpth_data = data_container->host_data.dpth_rigid_rigid;
  custom_vector<real>& erad_data = data_container->host_data.erad_rigid_rigid;
  custom_vector<int2>& bids_data = data_container->host_data.bids_rigid_rigid;
  //======== Collision model data
  custom_vector<int>& obj_data_T = data_container->host_data.typ_rigid;
  custom_vector<real3>& obj_data_A = data_container->host_data.ObA_rigid;
  custom_vector<real3>& obj_data_B = data_container->host_data.ObB_rigid;
  custom_vector<real3>& obj_data_C = data_container->host_data.ObC_rigid;
  custom_vector<real4>& obj_data_R = data_container->host_data.ObR_rigid;
  custom_vector<uint>& obj_data_ID = data_container->host_data.id_rigid;
  custom_vector<real3>& convex_data = data_container->host_data.convex_data;
  //======== Body state information
  custom_vector<bool>& obj_active = data_container->host_data.active_data;
  custom_vector<real3>& body_pos = data_container->host_data.pos_data;
  custom_vector<real4>& body_rot = data_container->host_data.rot_data;
  //======== Broadphase information
  custom_vector<long long>& potentialCollisions = data_container->host_data.pair_rigid_rigid;
  //======== Indexing variables and other information
  collision_envelope = data_container->settings.collision.collision_envelope;
  uint& number_of_contacts = data_container->num_contacts;
  narrowphase_algorithm = data_container->settings.collision.narrowphase_algorithm;
  system_type = data_container->settings.system_type;
  // The number of possible contacts based on the broadphase pair list
  num_potentialCollisions = potentialCollisions.size();

  // Return now if no potential collisions.
  if (num_potentialCollisions == 0) {
    norm_data.resize(0);
    cpta_data.resize(0);
    cptb_data.resize(0);
    dpth_data.resize(0);
    erad_data.resize(0);
    bids_data.resize(0);
    number_of_contacts = 0;
    return;
  }

  uint num_shapes = obj_data_T.size();

  obj_data_A_global = obj_data_A;      //.resize(num_shapes);
  obj_data_B_global = obj_data_B;      //.resize(num_shapes);
  obj_data_C_global = obj_data_C;      //.resize(num_shapes);
  convex_data_global = convex_data;    //.resize(num_shapes); //NOT USED RIGHT NOW
  // Transform to global coordinate system
  PreprocessLocalToParent(num_shapes,
                          obj_data_T.data(),
                          obj_data_A.data(),
                          obj_data_B.data(),
                          obj_data_C.data(),
                          obj_data_R.data(),
                          obj_data_ID.data(),
                          obj_active.data(),
                          body_pos.data(),
                          body_rot.data(),
                          obj_data_A_global.data(),
                          obj_data_B_global.data(),
                          obj_data_C_global.data(),
                          convex_data_global.data());

  contact_index.resize(num_potentialCollisions);

  // Count Number of Contacts
  PreprocessCount(obj_data_T.data(), potentialCollisions.data(), contact_index.data());
  // scan to find starting index
  int num_potentialContacts = contact_index.back();
  thrust::exclusive_scan(thrust_parallel, contact_index.begin(), contact_index.end(), contact_index.begin());
  num_potentialContacts += contact_index.back();

  // This counter will keep track of which pairs are actually in contact
  contact_active.resize(num_potentialContacts);
  // Fill the counter with 1, if the contact is active set the value to zero
  // POSSIBLE PERF IMPROVEMENT:, use bool for this?
  thrust::fill(contact_active.begin(), contact_active.end(), 1);
  // Create storage to hold maximum number of contacts in worse case
  norm_data.resize(num_potentialContacts);
  cpta_data.resize(num_potentialContacts);
  cptb_data.resize(num_potentialContacts);
  dpth_data.resize(num_potentialContacts);
  erad_data.resize(num_potentialContacts);
  bids_data.resize(num_potentialContacts);

  Dispatch(obj_data_T.data(),
           obj_data_A_global.data(),
           obj_data_B_global.data(),
           obj_data_C_global.data(),
           obj_data_R.data(),
           obj_data_ID.data(),
           convex_data.data(),
           obj_active.data(),
           body_pos.data(),
           body_rot.data(),
           potentialCollisions.data(),
           contact_index.data(),
           contact_active.data(),
           norm_data.data(),
           cpta_data.data(),
           cptb_data.data(),
           dpth_data.data(),
           erad_data.data(),
           bids_data.data());

  number_of_contacts = num_potentialContacts - thrust::count(contact_active.begin(), contact_active.end(), 1);

  // remove any entries where the counter is equal to one, these are contacts that do not exist
  thrust::remove_if(norm_data.begin(), norm_data.end(), contact_active.begin(), thrust::identity<int>());
  thrust::remove_if(cpta_data.begin(), cpta_data.end(), contact_active.begin(), thrust::identity<int>());
  thrust::remove_if(cptb_data.begin(), cptb_data.end(), contact_active.begin(), thrust::identity<int>());
  thrust::remove_if(dpth_data.begin(), dpth_data.end(), contact_active.begin(), thrust::identity<int>());
  thrust::remove_if(erad_data.begin(), erad_data.end(), contact_active.begin(), thrust::identity<int>());
  thrust::remove_if(bids_data.begin(), bids_data.end(), contact_active.begin(), thrust::identity<int>());
  thrust::remove_if(potentialCollisions.begin(), potentialCollisions.end(), contact_active.begin(), thrust::identity<int>());
  // Resize all lists so that we don't access invalid contacts
  potentialCollisions.resize(number_of_contacts);
  norm_data.resize(number_of_contacts);
  cpta_data.resize(number_of_contacts);
  cptb_data.resize(number_of_contacts);
  dpth_data.resize(number_of_contacts);
  erad_data.resize(number_of_contacts);
  bids_data.resize(number_of_contacts);
  data_container->erad_is_set = true;

  // std::cout << num_potentialContacts << " " << number_of_contacts << std::endl;
}

void host_count(const int& index,                                // index of this potential collision
                const shape_type* obj_data_T,                    // shape type (per shape)
                const long long* collision_pair,                 // encoded shape IDs (per collision pair)
                const NARROWPHASETYPE& narrowphase_algorithm,    // Type of narrowphase being used
                uint* max_contacts)                              // max. number of contacts (per collision pair)
{
  // Identify the two candidate shapes and get their types.
  int2 pair = I2(int(collision_pair[index] >> 32), int(collision_pair[index] & 0xffffffff));
  shape_type type1 = obj_data_T[pair.x];
  shape_type type2 = obj_data_T[pair.y];

  if (narrowphase_algorithm == NARROWPHASE_MPR) {
    max_contacts[index] = 1;
    return;
    //} else if (narrowphase_algorithm == NARROWPHASE_GJK) {
    //   max_contacts[index] = 1;
    //   return;
  } else if (narrowphase_algorithm == NARROWPHASE_HYBRID_MPR) {
    max_contacts[index] = 1;
    if (type1 == SPHERE || type2 == SPHERE) {
      max_contacts[index] = 1;
    } else if (type1 == CAPSULE || type2 == CAPSULE) {
      max_contacts[index] = 2;
    }
    return;
    //} else if (narrowphase_algorithm == NARROWPHASE_HYBRID_GJK) {
    //   max_contacts[index] = 1;
    //   if (type1 == SPHERE || type2 == SPHERE) {
    //      max_contacts[index] = 1;
    //   } else if (type1 == CAPSULE || type2 == CAPSULE) {
    //      max_contacts[index] = 2;
    //   }
  }

  // Set the maximum number of possible contacts for this particular pair
  if (type1 == SPHERE || type2 == SPHERE) {
    max_contacts[index] = 1;
  } else if (type1 == CAPSULE || type2 == CAPSULE) {
    max_contacts[index] = 2;
  } else {
    max_contacts[index] = 4;
  }
}
void ChCNarrowphaseDispatch::PreprocessCount(const shape_type* obj_data_T,       // shape type (per shape)
                                             const long long* collision_pair,    // encoded shape IDs (per collision pair)
                                             uint* max_contacts) {
#pragma omp parallel for
  for (int icoll = 0; icoll < num_potentialCollisions; icoll++) {
    host_count(icoll, obj_data_T, collision_pair, narrowphase_algorithm, max_contacts);
  }
}
void host_Preprocess(const uint& index,
                     const shape_type* obj_data_T,
                     const real3* obj_data_A,
                     const real3* obj_data_B,
                     const real3* obj_data_C,
                     const uint* obj_data_ID,
                     const real3* body_pos,
                     const real4* body_rot,
                     real3* obj_data_A_global,
                     real3* obj_data_B_global,
                     real3* obj_data_C_global) {

  shape_type T = obj_data_T[index];

  // Get the identifier for the object associated with this collision shape
  uint ID = obj_data_ID[index];

  real3 pos = body_pos[ID];    // Get the global object position
  real4 rot = body_rot[ID];    // Get the global object rotation

  obj_data_A_global[index] = TransformLocalToParent(pos, rot, obj_data_A[index]);
  if (T == TRIANGLEMESH) {
    obj_data_B_global[index] = TransformLocalToParent(pos, rot, obj_data_B[index]);
    obj_data_C_global[index] = TransformLocalToParent(pos, rot, obj_data_C[index]);
  }
}

void ChCNarrowphaseDispatch::PreprocessLocalToParent(const int num_shapes,
                                                     const shape_type* obj_data_T,
                                                     const real3* obj_data_A,
                                                     const real3* obj_data_B,
                                                     const real3* obj_data_C,
                                                     const real4* obj_data_R,
                                                     const uint* obj_data_ID,
                                                     const bool* obj_active,
                                                     const real3* body_pos,
                                                     const real4* body_rot,
                                                     real3* obj_data_A_global,
                                                     real3* obj_data_B_global,
                                                     real3* obj_data_C_global,
                                                     real3* convex_data_mod) {
#pragma omp parallel for
  for (int index = 0; index < num_shapes; index++) {
    host_Preprocess(index, obj_data_T, obj_data_A, obj_data_B, obj_data_C, obj_data_ID, body_pos, body_rot, obj_data_A_global, obj_data_B_global, obj_data_C_global);
  }
}

bool host_Dispatch_Init(const uint& index,
                        const shape_type* obj_data_T,
                        const real3* obj_data_A,
                        const real3* obj_data_B,
                        const real3* obj_data_C,
                        const real4* obj_data_R,
                        const uint* obj_data_ID,
                        real3* convex_data,
                        const bool* obj_active,
                        const real3* body_pos,
                        const real4* body_rot,
                        const long long* contact_pair,
                        const uint* start_index,
                        uint& icoll,
                        uint& ID_A,
                        uint& ID_B,
                        ConvexShape& shapeA,
                        ConvexShape& shapeB) {
  long long p = contact_pair[index];
  int2 pair = I2(int(p >> 32), int(p & 0xffffffff));
  ID_A = obj_data_ID[pair.x];
  ID_B = obj_data_ID[pair.y];

  if (obj_active[ID_A] == false && obj_active[ID_B] == false) {
    return false;
  }
  if (ID_A == ID_B) {
    return false;
  }

  shapeA.type = obj_data_T[pair.x];
  shapeB.type = obj_data_T[pair.y];    // Get the type data for each object in the collision pair

  real4 rotA = (body_rot[ID_A]), rotB = (body_rot[ID_B]);    // Get the global object rotation
  shapeA.A = obj_data_A[pair.x];
  shapeB.A = obj_data_A[pair.y];
  shapeA.B = obj_data_B[pair.x];
  shapeB.B = obj_data_B[pair.y];
  shapeA.C = obj_data_C[pair.x];
  shapeB.C = obj_data_C[pair.y];
  shapeA.R = (mult(rotA, obj_data_R[pair.x]));
  shapeB.R = (mult(rotB, obj_data_R[pair.y]));
  shapeA.convex = convex_data;
  shapeB.convex = convex_data;

  //// TODO: what is the best way to dispatch this?
  icoll = start_index[index];
  return true;
}

void host_Dispatch_Finalize(const uint& icoll,
                            const real3* body_pos,
                            const real& collision_envelope,
                            const SYSTEMTYPE& system_type,
                            const uint& ID_A,
                            const uint& ID_B,
                            const int& nC,
                            real3* norm,
                            real3* ptA,
                            real3* ptB,
                            real* contactDepth) {
  if (system_type == SYSTEM_DVI) {
    // perform offset for DVI

    real3 posA = body_pos[ID_A], posB = body_pos[ID_B];    // Get the global object position

    for (int i = 0; i < nC; i++) {
      ptA[icoll + i] = ptA[icoll + i] - (norm[icoll + i]) * collision_envelope - posA;
      ptB[icoll + i] = ptB[icoll + i] + (norm[icoll + i]) * collision_envelope - posB;

      contactDepth[icoll + i] += collision_envelope * 2;

      //         std::cout << "N" << norm[icoll + i];
      //         std::cout << "A" << ptA[icoll + i];
      //         std::cout << "B" << ptB[icoll + i];
      //         std::cout << "d " << contactDepth[icoll + i] - collision_envelope * 2 << " " << contactDepth[icoll + i] << std::endl;
    }
  }
}

void host_DispatchMPR(const uint& index,
                      const shape_type* obj_data_T,
                      const real3* obj_data_A,
                      const real3* obj_data_B,
                      const real3* obj_data_C,
                      const real4* obj_data_R,
                      const uint* obj_data_ID,
                      real3* convex_data,
                      const bool* obj_active,
                      const real3* body_pos,
                      const real4* body_rot,
                      const real& collision_envelope,
                      const SYSTEMTYPE& system_type,
                      const long long* contact_pair,
                      const uint* start_index,
                      uint* flag,
                      real3* norm,
                      real3* ptA,
                      real3* ptB,
                      real* contactDepth,
                      real* effective_radius,
                      int2* body_ids) {

  uint ID_A, ID_B, icoll;
  ConvexShape shapeA, shapeB;

  if (!host_Dispatch_Init(
           index, obj_data_T, obj_data_A, obj_data_B, obj_data_C, obj_data_R, obj_data_ID, convex_data, obj_active, body_pos, body_rot, contact_pair, start_index, icoll, ID_A, ID_B, shapeA, shapeB)) {
    return;
  }
  int nC = 0;
  if (!MPRCollision(shapeA, shapeB, collision_envelope, norm[icoll], ptA[icoll], ptB[icoll], contactDepth[icoll])) {
    return;
  }
  effective_radius[icoll] = edge_radius;
  body_ids[icoll] = I2(ID_A, ID_B);
  flag[icoll] = 0;
  nC = 1;
  host_Dispatch_Finalize(icoll, body_pos, collision_envelope, system_type, ID_A, ID_B, nC, norm, ptA, ptB, contactDepth);
}

void host_DispatchR(const uint& index,
                    const shape_type* obj_data_T,
                    const real3* obj_data_A,
                    const real3* obj_data_B,
                    const real3* obj_data_C,
                    const real4* obj_data_R,
                    const uint* obj_data_ID,
                    real3* convex_data,
                    const bool* obj_active,
                    const real3* body_pos,
                    const real4* body_rot,
                    const real& collision_envelope,
                    const SYSTEMTYPE& system_type,
                    const long long* contact_pair,
                    const uint* start_index,
                    uint* flag,
                    real3* norm,
                    real3* ptA,
                    real3* ptB,
                    real* contactDepth,
                    real* effective_radius,
                    int2* body_ids) {
  uint ID_A, ID_B, icoll;
  ConvexShape shapeA, shapeB;

  if (!host_Dispatch_Init(
           index, obj_data_T, obj_data_A, obj_data_B, obj_data_C, obj_data_R, obj_data_ID, convex_data, obj_active, body_pos, body_rot, contact_pair, start_index, icoll, ID_A, ID_B, shapeA, shapeB)) {
    return;
  }
  int nC = 0;
  if (!RCollision(icoll, shapeA, shapeB, ID_A, ID_B, flag, norm, ptA, ptB, contactDepth, effective_radius, body_ids, nC)) {
    return;
  }

  host_Dispatch_Finalize(icoll, body_pos, collision_envelope, system_type, ID_A, ID_B, nC, norm, ptA, ptB, contactDepth);
}

void host_DispatchHybridMPR(const uint& index,
                            const shape_type* obj_data_T,
                            const real3* obj_data_A,
                            const real3* obj_data_B,
                            const real3* obj_data_C,
                            const real4* obj_data_R,
                            const uint* obj_data_ID,
                            real3* convex_data,
                            const bool* obj_active,
                            const real3* body_pos,
                            const real4* body_rot,
                            const real& collision_envelope,
                            const SYSTEMTYPE& system_type,
                            const long long* contact_pair,
                            const uint* start_index,
                            uint* flag,
                            real3* norm,
                            real3* ptA,
                            real3* ptB,
                            real* contactDepth,
                            real* effective_radius,
                            int2* body_ids) {
  uint ID_A, ID_B, icoll;
  ConvexShape shapeA, shapeB;

  if (!host_Dispatch_Init(
           index, obj_data_T, obj_data_A, obj_data_B, obj_data_C, obj_data_R, obj_data_ID, convex_data, obj_active, body_pos, body_rot, contact_pair, start_index, icoll, ID_A, ID_B, shapeA, shapeB)) {
    return;
  }
  int nC = 0;

  if (RCollision(icoll, shapeA, shapeB, ID_A, ID_B, flag, norm, ptA, ptB, contactDepth, effective_radius, body_ids, nC)) {
    // this is needed for DVI

  } else {

    if (!MPRCollision(shapeA, shapeB, collision_envelope, norm[icoll], ptA[icoll], ptB[icoll], contactDepth[icoll])) {
      return;
    }

    effective_radius[icoll] = edge_radius;
    body_ids[icoll] = I2(ID_A, ID_B);
    flag[icoll] = 0;

    nC = 1;
  }

  host_Dispatch_Finalize(icoll, body_pos, collision_envelope, system_type, ID_A, ID_B, nC, norm, ptA, ptB, contactDepth);
}

void ChCNarrowphaseDispatch::Dispatch(const shape_type* obj_data_T,
                                      const real3* obj_data_A,
                                      const real3* obj_data_B,
                                      const real3* obj_data_C,
                                      const real4* obj_data_R,
                                      const uint* obj_data_ID,
                                      real3* convex_data,
                                      const bool* obj_active,
                                      const real3* body_pos,
                                      const real4* body_rot,
                                      const long long* contact_pair,
                                      const uint* start_index,
                                      uint* contact_active,
                                      real3* norm,
                                      real3* ptA,
                                      real3* ptB,
                                      real* contactDepth,
                                      real* erad,
                                      int2* ids) {

  switch (narrowphase_algorithm) {
    case NARROWPHASE_MPR:
#pragma omp parallel for
      for (int index = 0; index < num_potentialCollisions; index++) {
        host_DispatchMPR(index,
                         obj_data_T,
                         obj_data_A,
                         obj_data_B,
                         obj_data_C,
                         obj_data_R,
                         obj_data_ID,
                         convex_data,
                         obj_active,
                         body_pos,
                         body_rot,
                         collision_envelope,
                         system_type,
                         contact_pair,
                         start_index,
                         contact_active,
                         norm,
                         ptA,
                         ptB,
                         contactDepth,
                         erad,
                         ids);
      }
      break;
    case NARROWPHASE_R:
#pragma omp parallel for
      for (int index = 0; index < num_potentialCollisions; index++) {
        host_DispatchR(index,
                       obj_data_T,
                       obj_data_A,
                       obj_data_B,
                       obj_data_C,
                       obj_data_R,
                       obj_data_ID,
                       convex_data,
                       obj_active,
                       body_pos,
                       body_rot,
                       collision_envelope,
                       system_type,
                       contact_pair,
                       start_index,
                       contact_active,
                       norm,
                       ptA,
                       ptB,
                       contactDepth,
                       erad,
                       ids);
      }
      break;
    case NARROWPHASE_HYBRID_MPR:
#pragma omp parallel for
      for (int index = 0; index < num_potentialCollisions; index++) {
        host_DispatchHybridMPR(index,
                               obj_data_T,
                               obj_data_A,
                               obj_data_B,
                               obj_data_C,
                               obj_data_R,
                               obj_data_ID,
                               convex_data,
                               obj_active,
                               body_pos,
                               body_rot,
                               collision_envelope,
                               system_type,
                               contact_pair,
                               start_index,
                               contact_active,
                               norm,
                               ptA,
                               ptB,
                               contactDepth,
                               erad,
                               ids);
      }
      break;
  }
}
}
}
