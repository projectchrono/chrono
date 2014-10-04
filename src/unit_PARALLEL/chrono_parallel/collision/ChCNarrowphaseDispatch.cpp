#include <algorithm>

#include "collision/ChCCollisionModel.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "ChCNarrowphaseDispatch.h"
#include "ChCNarrowphaseMPRUtils.h"

using namespace chrono::collision;

void ChCNarrowphaseDispatch::Process(ChParallelDataManager* data_container) {

   PerformNarrowphase(data_container->host_data.typ_rigid, data_container->host_data.ObA_rigid, data_container->host_data.ObB_rigid, data_container->host_data.ObC_rigid,
                      data_container->host_data.ObR_rigid, data_container->host_data.id_rigid, data_container->host_data.active_data, data_container->host_data.pos_data,
                      data_container->host_data.rot_data, data_container->settings.collision.collision_envelope, data_container->settings.collision.narrowphase_algorithm,
                      data_container->host_data.pair_rigid_rigid, data_container->host_data.norm_rigid_rigid, data_container->host_data.cpta_rigid_rigid,
                      data_container->host_data.cptb_rigid_rigid, data_container->host_data.dpth_rigid_rigid, data_container->host_data.erad_rigid_rigid,
                      data_container->host_data.bids_rigid_rigid, data_container->num_contacts);

   data_container->erad_is_set = true;

}

void host_count(const int& index,                   // index of this potential collision
                const shape_type* obj_data_T,       // shape type (per shape)
                const long long* collision_pair,    // encoded shape IDs (per collision pair)
                const NARROWPHASETYPE &narrowphase_algorithm,  //Type of narrowphase being used
                uint* max_contacts)                 // max. number of contacts (per collision pair)
                {
   // Identify the two candidate shapes and get their types.
   int2 pair = I2(int(collision_pair[index] >> 32), int(collision_pair[index] & 0xffffffff));
   shape_type type1 = obj_data_T[pair.x];
   shape_type type2 = obj_data_T[pair.y];

   if (narrowphase_algorithm == NARROWPHASE_MPR) {
      max_contacts[index] = 1;
      return;
   } else if (narrowphase_algorithm == NARROWPHASE_GJK) {
      max_contacts[index] = 1;
      return;
   } else if (narrowphase_algorithm == NARROWPHASE_HYBRID_MPR) {
      max_contacts[index] = 1;
   } else if (narrowphase_algorithm == NARROWPHASE_HYBRID_GJK) {
      max_contacts[index] = 1;
   }

// Set the maximum number of possible contacts for this particular pair
   if (type1 == SPHERE || type2 == SPHERE)
      max_contacts[index] = 1;
   else if (type1 == CAPSULE || type2 == CAPSULE)
      max_contacts[index] = 2;
   else
      max_contacts[index] = 4;
}
void ChCNarrowphaseDispatch::PreprocessCount(const shape_type* obj_data_T,       // shape type (per shape)
                                             const long long* collision_pair,    // encoded shape IDs (per collision pair)
                                             const NARROWPHASETYPE &narrowphase_algorithm,  //Type of narrowphase being used
                                             uint* max_contacts) {
#pragma omp parallel for
   for (int icoll = 0; icoll < num_potentialCollisions; icoll++) {
      host_count(icoll, obj_data_T, collision_pair, narrowphase_algorithm, max_contacts);
   }
}
void host_Preprocess(const uint &index,
                     const shape_type *obj_data_T,
                     const real3 *obj_data_A,
                     const real3 *obj_data_B,
                     const real3 *obj_data_C,
                     const real4 *obj_data_R,
                     const uint *obj_data_ID,
                     const bool * obj_active,
                     const real3 *body_pos,
                     const real4 *body_rot,
                     real3 *obj_data_A_global,
                     real3 *obj_data_B_global,
                     real3 *obj_data_C_global) {

   shape_type T = obj_data_T[index];

//Get the identifier for the object associated with this collision shape
   uint ID = obj_data_ID[index];

   real3 pos = body_pos[ID];     //Get the global object position
   real4 rot = body_rot[ID];     //Get the global object rotation

   obj_data_A_global[index] = TransformLocalToParent(pos, rot, obj_data_A[index]);
   if (T == TRIANGLEMESH) {
      obj_data_B_global[index] = TransformLocalToParent(pos, rot, obj_data_B[index]);
      obj_data_C_global[index] = TransformLocalToParent(pos, rot, obj_data_C[index]);
   }

}

void ChCNarrowphaseDispatch::PreprocessLocalToParent(const int num_shapes,
                                                     const shape_type *obj_data_T,
                                                     const real3 *obj_data_A,
                                                     const real3 *obj_data_B,
                                                     const real3 *obj_data_C,
                                                     const real4 *obj_data_R,
                                                     const uint *obj_data_ID,
                                                     const bool * obj_active,
                                                     const real3 *body_pos,
                                                     const real4 *body_rot,
                                                     real3 *obj_data_A_global,
                                                     real3 *obj_data_B_global,
                                                     real3 *obj_data_C_global) {
#pragma omp parallel for
   for (int index = 0; index < num_shapes; index++) {
      host_Preprocess(index, obj_data_T, obj_data_A, obj_data_B, obj_data_C, obj_data_R, obj_data_ID, obj_active, body_pos, body_rot, obj_data_A_global, obj_data_B_global,
                      obj_data_C_global);
   }
}

bool host_DispatchMPR(const ConvexShape & shapeA,
                      const ConvexShape & shapeB,
                      real3 &normal,
                      real & depth,
                      real3 & pointA,
                      real3 & pointB) {

   real3 point = R3(0);
   normal = R3(1, 0, 0);
   pointA = R3(0);
   pointB = R3(0);
   depth = 0;

   if (!MPRCollision(shapeA, shapeB, normal, point, depth)) {
      return false;
   }

   MPRGetPoints(shapeA, shapeB, normal, point, pointA, pointB);

   return true;
}

bool host_DispatchR(const uint& icoll,
                    const ConvexShape &shapeA,
                    const ConvexShape &shapeB,
                    const uint* start_index,
                    const int& body1,
                    const int& body2,
                    uint* flag,
                    real3* normal,
                    real3* pointA,
                    real3* pointB,
                    real* depth,
                    real* effective_radius,
                    int2* body_ids) {

   real3 point = R3(0);
   normal = R3(1, 0, 0);
   pointA = R3(0);
   pointB = R3(0);
   depth = 0;

   if (!RCollision(shapeA, shapeB, normal, point, depth)) {
      return false;
   }

   return true;
}

void host_Dispatch(const uint &index,
                   const shape_type *obj_data_T,
                   const real3 *obj_data_A,
                   const real3 *obj_data_B,
                   const real3 *obj_data_C,
                   const real4 *obj_data_R,
                   const uint *obj_data_ID,
                   const bool * obj_active,
                   const real3 *body_pos,
                   const real4 *body_rot,
                   const real & collision_envelope,
                   const NARROWPHASETYPE &narrowphase_algorithm,
                   long long *contact_pair,
                   uint *contact_active,
                   real3 *norm,
                   real3 *ptA,
                   real3 *ptB,
                   real *contactDepth,
                   int2 *ids,
                   uint * start_index) {
   long long p = contact_pair[index];
   int2 pair = I2(int(p >> 32), int(p & 0xffffffff));
   uint ID_A = obj_data_ID[pair.x];
   uint ID_B = obj_data_ID[pair.y];

   if (obj_active[ID_A] == false && obj_active[ID_B] == false) {
      return;
   }
   if (ID_A == ID_B) {
      return;
   }
   shape_type A_T = obj_data_T[pair.x], B_T = obj_data_T[pair.y];     //Get the type data for each object in the collision pair

   ConvexShape shapeA, shapeB;

   shapeA.type = A_T;
   shapeB.type = B_T;     //Get the type data for each object in the collision pair
   real3 posA = body_pos[ID_A], posB = body_pos[ID_B];     //Get the global object position
   real4 rotA = (body_rot[ID_A]), rotB = (body_rot[ID_B]);     //Get the global object rotation
   shapeA.A = obj_data_A[pair.x];
   shapeB.A = obj_data_A[pair.y];
   shapeA.B = obj_data_B[pair.x];
   shapeB.B = obj_data_B[pair.y];
   shapeA.C = obj_data_C[pair.x];
   shapeB.C = obj_data_C[pair.y];
   shapeA.R = (mult(rotA, obj_data_R[pair.x]));
   shapeB.R = (mult(rotB, obj_data_R[pair.y]));

   //// TODO: what is the best way to dispatch this?
   uint icoll = start_index[index];

   real3 normal;
   real3 pointA, pointB;
   real depth;
   switch (narrowphase_algorithm) {
      case NARROWPHASE_MPR:
         host_DispatchMPR(shapeA, shapeB, normal, depth, pointA, pointB);
         pointA -= (normal) * envelope;
         pointB -= (normal) * envelope;

         norm[icoll] = normal;
         ptA[icoll] = pointA - posA;
         ptB[icoll] = pointB - posB;
         contactDepth[icoll] = depth;
         ids[icoll] = I2(ID_A, ID_B);
         contact_active[icoll] = 0;


         break;
      case NARROWPHASE_GJK:
         //host_DispatchGJK(shapeA, shapeB, normal, depth, pointA, pointB);
         break;
      case NARROWPHASE_R:
         host_DispatchR(shapeA, shapeB, normal, depth, pointA, pointB);
         break;
      case NARROWPHASE_HYBRID_MPR:
         //host_DispatchHYBRID_MPR(shapeA, shapeB, normal, depth, pointA, pointB);
         break;
      case NARROWPHASE_HYBRID_GJK:
         //host_DispatchHYBRID_GJK(shapeA, shapeB, normal, depth, pointA, pointB);
         break;
   }



}

void ChCNarrowphaseDispatch::Dispatch(const shape_type *obj_data_T,
                                      const real3 *obj_data_A,
                                      const real3 *obj_data_B,
                                      const real3 *obj_data_C,
                                      const real4 *obj_data_R,
                                      const uint *obj_data_ID,
                                      const bool * obj_active,
                                      const real3 *body_pos,
                                      const real4 *body_rot,
                                      const real & collision_envelope,
                                      const NARROWPHASETYPE &narrowphase_algorithm,
                                      long long *contact_pair,
                                      uint *contact_active,
                                      real3 *norm,
                                      real3 *ptA,
                                      real3 *ptB,
                                      real *contactDepth,
                                      int2 *ids,
                                      uint* start_index) {
#pragma omp parallel for
   for (int index = 0; index < num_potentialCollisions; index++) {
      host_Dispatch(index, obj_data_T, obj_data_A, obj_data_B, obj_data_C, obj_data_R, obj_data_ID, obj_active, body_pos, body_rot, collision_envelope, narrowphase_algorithm,
                    contact_pair, contact_active, norm, ptA, ptB, contactDepth, ids, start_index);
   }
}

void ChCNarrowphaseDispatch::PerformNarrowphase(const custom_vector<shape_type> &obj_data_T,
                                                const custom_vector<real3> &obj_data_A,
                                                const custom_vector<real3> &obj_data_B,
                                                const custom_vector<real3> &obj_data_C,
                                                const custom_vector<real4> &obj_data_R,
                                                const custom_vector<uint> &obj_data_ID,
                                                const custom_vector<bool> & obj_active,
                                                const custom_vector<real3> &body_pos,
                                                const custom_vector<real4> &body_rot,
                                                const real & collision_envelope,
                                                const NARROWPHASETYPE &narrowphase_algorithm,
                                                custom_vector<long long> &potentialCollisions,
                                                custom_vector<real3> &norm_data,
                                                custom_vector<real3> &cpta_data,
                                                custom_vector<real3> &cptb_data,
                                                custom_vector<real> &dpth_data,
                                                custom_vector<real> &erad_data,
                                                custom_vector<int2> &bids_data,
                                                uint & number_of_contacts) {
//The number of possible contacts based on the broadphase pair list
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

   obj_data_A_global.resize(num_shapes);
   obj_data_B_global.resize(num_shapes);
   obj_data_C_global.resize(num_shapes);
//Transform to global coordinate system
   PreprocessLocalToParent(num_shapes, obj_data_T.data(), obj_data_A.data(), obj_data_B.data(), obj_data_C.data(), obj_data_R.data(), obj_data_ID.data(), obj_active.data(),
                           body_pos.data(), body_rot.data(), obj_data_A_global.data(), obj_data_B_global.data(), obj_data_C_global.data());

   custom_vector<uint> contact_index(num_potentialCollisions);

//Count Number of Contacts
   PreprocessCount(obj_data_T.data(), potentialCollisions.data(), narrowphase_algorithm, contact_index.data());
//scan to find starting index
   int num_potentialContacts = contact_index.back();
   thrust::exclusive_scan(contact_index.begin(), contact_index.end(), contact_index.begin());
   num_potentialContacts += contact_index.back();

//This counter will keep track of which pairs are actually in contact
   contact_active.resize(num_potentialCollisions);
//Fill the counter with 1, if the contact is active set the value to zero
//POSSIBLE PERF IMPROVEMENT:, use bool for this?
   thrust::fill(contact_active.begin(), contact_active.end(), 1);
//Create storage to hold maximum number of contacts in worse case
   norm_data.resize(num_potentialCollisions);
   cpta_data.resize(num_potentialCollisions);
   cptb_data.resize(num_potentialCollisions);
   dpth_data.resize(num_potentialCollisions);
   bids_data.resize(num_potentialCollisions);

   Dispatch(obj_data_T.data(), obj_data_A_global.data(), obj_data_B_global.data(), obj_data_C_global.data(), obj_data_R.data(), obj_data_ID.data(), obj_active.data(),
            body_pos.data(), body_rot.data(), collision_envelope, narrowphase_algorithm, potentialCollisions.data(), contact_active.data(), norm_data.data(), cpta_data.data(),
            cptb_data.data(), dpth_data.data(), bids_data.data(), contact_index.data());

   number_of_contacts = num_potentialCollisions - thrust::count(contact_active.begin(), contact_active.end(), 1);
//remove any entries where the counter is equal to one, these are contacts that do not exist
   thrust::remove_if(norm_data.begin(), norm_data.end(), contact_active.begin(), thrust::identity<int>());
   thrust::remove_if(cpta_data.begin(), cpta_data.end(), contact_active.begin(), thrust::identity<int>());
   thrust::remove_if(cptb_data.begin(), cptb_data.end(), contact_active.begin(), thrust::identity<int>());
   thrust::remove_if(dpth_data.begin(), dpth_data.end(), contact_active.begin(), thrust::identity<int>());
   thrust::remove_if(bids_data.begin(), bids_data.end(), contact_active.begin(), thrust::identity<int>());
   thrust::remove_if(potentialCollisions.begin(), potentialCollisions.end(), contact_active.begin(), thrust::identity<int>());
//Resize all lists so that we don't access invalid contacts
   potentialCollisions.resize(number_of_contacts);
   norm_data.resize(number_of_contacts);
   cpta_data.resize(number_of_contacts);
   cptb_data.resize(number_of_contacts);
   dpth_data.resize(number_of_contacts);
   bids_data.resize(number_of_contacts);

}
