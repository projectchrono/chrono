#ifndef CHC_NARROWPHASE_MPR_H
#define CHC_NARROWPHASE_MPR_H

#include "chrono_parallel/collision/ChCNarrowphase.h"

namespace chrono {
namespace collision {

CH_PARALLEL_API
bool CollideAndFindPoint(
      const shape_type &typeA,
      const real3 &A_X,
      const real3 &A_Y,
      const real3 &A_Z,
      const real4 &A_R,
      const shape_type &typeB,
      const real3 &B_X,
      const real3 &B_Y,
      const real3 &B_Z,
      const real4 &B_R,
      real3 &returnNormal,
      real3 &point,
      real &depth);

CH_PARALLEL_API
void GetPoints(
      shape_type A_T,
      real3 A_X,
      real3 A_Y,
      real3 A_Z,
      real4 A_R,
      shape_type B_T,
      real3 B_X,
      real3 B_Y,
      real3 B_Z,
      real4 B_R,
      real3 &N,
      real3 p0,
      real3 & p1,
      real3 & p2);

CH_PARALLEL_API
bool SphereSphere(
      const real3 &A_X,
      const real3 &B_X,
      const real3 &A_Y,
      const real3 &B_Y,
      real3 & N,
      real & dist,
      real3 & p1,
      real3 & p2);


// Minkovski Portal Refinement narrowphase collision detection
class CH_PARALLEL_API ChCNarrowphaseMPR : public ChCNarrowphase {
 public:
   ChCNarrowphaseMPR() {
   }

   virtual void Process(
         ChParallelDataManager* data_container) {
      DoNarrowphase(data_container->host_data.typ_rigid, data_container->host_data.ObA_rigid, data_container->host_data.ObB_rigid, data_container->host_data.ObC_rigid,
                    data_container->host_data.ObR_rigid, data_container->host_data.id_rigid, data_container->host_data.active_data, data_container->host_data.pos_data,
                    data_container->host_data.rot_data, data_container->host_data.pair_rigid_rigid, data_container->host_data.norm_rigid_rigid,
                    data_container->host_data.cpta_rigid_rigid, data_container->host_data.cptb_rigid_rigid, data_container->host_data.dpth_rigid_rigid,
                    data_container->host_data.bids_rigid_rigid, data_container->num_contacts);
   }

   virtual void Update(
         ChParallelDataManager* data_container) {
      UpdateNarrowphase(data_container->host_data.typ_rigid, data_container->host_data.ObA_rigid, data_container->host_data.ObB_rigid, data_container->host_data.ObC_rigid,
                        data_container->host_data.ObR_rigid, data_container->host_data.id_rigid, data_container->host_data.active_data, data_container->host_data.pos_new_data,
                        data_container->host_data.rot_new_data, data_container->num_contacts, data_container->host_data.norm_rigid_rigid,
                        data_container->host_data.cpta_rigid_rigid, data_container->host_data.cptb_rigid_rigid, data_container->host_data.dpth_rigid_rigid,
                        data_container->host_data.bids_rigid_rigid);
   }

   void function_MPR_Store(
         const uint &index,
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
         long long *contact_pair,
         uint *contact_active,
         real3 *norm,
         real3 *ptA,
         real3 *ptB,
         real *contactDepth,
         int2 *ids);

   void function_MPR_Update(
         const uint &index,
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
         real3 *norm,
         real3 *ptA,
         real3 *ptB,
         real *contactDepth,
         int2 *ids);
 private:
   void DoNarrowphase(
         const custom_vector<shape_type> &obj_data_T,
         const custom_vector<real3> &obj_data_A,
         const custom_vector<real3> &obj_data_B,
         const custom_vector<real3> &obj_data_C,
         const custom_vector<real4> &obj_data_R,
         const custom_vector<uint> &obj_data_ID,
         const custom_vector<bool> & obj_active,
         const custom_vector<real3> &body_pos,
         const custom_vector<real4> &body_rot,
         custom_vector<long long> &potentialCollisions,
         custom_vector<real3> &norm_data,
         custom_vector<real3> &cpta_data,
         custom_vector<real3> &cptb_data,
         custom_vector<real> &dpth_data,
         custom_vector<int2> &bids_data,
         uint & number_of_contacts);

   void UpdateNarrowphase(
         const custom_vector<shape_type> &obj_data_T,
         const custom_vector<real3> &obj_data_A,
         const custom_vector<real3> &obj_data_B,
         const custom_vector<real3> &obj_data_C,
         const custom_vector<real4> &obj_data_R,
         const custom_vector<uint> &obj_data_ID,
         const custom_vector<bool> & obj_active,
         const custom_vector<real3> &body_pos,
         const custom_vector<real4> &body_rot,
         const uint & number_of_contacts,
         custom_vector<real3> &norm_data,
         custom_vector<real3> &cpta_data,
         custom_vector<real3> &cptb_data,
         custom_vector<real> &dpth_data,
         custom_vector<int2> &bids_data);

   void host_MPR_Store(
         const shape_type *obj_data_T,
         const real3 *obj_data_A,
         const real3 *obj_data_B,
         const real3 *obj_data_C,
         const real4 *obj_data_R,
         const uint *obj_data_ID,
         const bool * obj_active,
         const real3 *body_pos,
         const real4 *body_rot,
         long long *contact_pair,
         uint *contact_active,
         real3 *norm,
         real3 *ptA,
         real3 *ptB,
         real *contactDepth,
         int2 *ids);

   void host_MPR_Update(
         const shape_type *obj_data_T,
         const real3 *obj_data_A,
         const real3 *obj_data_B,
         const real3 *obj_data_C,
         const real4 *obj_data_R,
         const uint *obj_data_ID,
         const bool * obj_active,
         const real3 *body_pos,
         const real4 *body_rot,
         real3 *norm,
         real3 *ptA,
         real3 *ptB,
         real *contactDepth,
         int2 *ids);
};

}  // end namespace collision
}  // end namespace chrono

#endif

