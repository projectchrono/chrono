#ifndef CHCONSTRAINT_RIGIDRIGID_H
#define CHCONSTRAINT_RIGIDRIGID_H

#include "chrono_parallel/ChBaseParallel.h"

namespace chrono {

CH_PARALLEL_API
void Orthogonalize(
      real3 &U,
      real3 &V,
      real3 &W);

CH_PARALLEL_API
void Compute_Jacobian(
      const real4& quat,
      const real3& U,
      const real3& V,
      const real3& W,
      const real3& point,
      real3& T1,
      real3& T2,
      real3& T3);

CH_PARALLEL_API
void Compute_Jacobian_Rolling(
      const real4& quat,
      const real3& U,
      const real3& V,
      const real3& W,
      real3& T1,
      real3& T2,
      real3& T3);

class CH_PARALLEL_API ChConstraintRigidRigid : public ChBaseParallel {
 public:
   ChConstraintRigidRigid() {
   }
   void Setup(
         ChParallelDataManager *data_container_) {

      data_container = data_container_;
      Initialize();
      num_updates = 0;
      inv_hpa = 1.0 / (step_size + alpha);
      inv_hhpa = 1.0 / (step_size * (step_size + alpha));

      if (num_contacts > 0) {
         update_number.resize((num_contacts) * 2, 0);
         offset_counter.resize((num_contacts) * 2, 0);
         update_offset.resize((num_contacts) * 2, 0);
         body_num.resize((num_contacts) * 2, 0);
         vel_update.resize((num_contacts) * 2);
         omg_update.resize((num_contacts) * 2);

         host_Offsets(data_container->host_data.bids_rigid_rigid.data(), body_num.data());

         thrust::sequence(update_number.begin(), update_number.end());
         thrust::sequence(update_offset.begin(), update_offset.end());
         thrust::fill(offset_counter.begin(), offset_counter.end(), 0);
         thrust::sort_by_key(thrust::omp::par, body_num.begin(), body_num.end(), update_number.begin());
         thrust::sort_by_key(thrust::omp::par, update_number.begin(), update_number.end(), update_offset.begin());
         body_number = body_num;
         num_updates = (thrust::reduce_by_key(body_num.begin(), body_num.end(), thrust::constant_iterator<uint>(1), update_number.begin(), offset_counter.begin()).second)
               - offset_counter.begin();
         thrust::inclusive_scan(offset_counter.begin(), offset_counter.end(), offset_counter.begin());

         update_offset_pairs.resize(int(num_contacts));
         update_offset_bodies.resize(int(num_contacts * 2));
         contact_active_pairs.resize(int(num_contacts));
         contact_rotation.resize(int(num_contacts * 2));

#pragma omp parallel for
         for (int i = 0; i < num_contacts; i++) {
            update_offset_pairs[i] = I2(update_offset[i], update_offset[i + num_contacts]);
            update_offset_bodies[update_offset[i]] = i;
            update_offset_bodies[update_offset[i + num_contacts]] = i + num_contacts;
            int2 body = data_container->host_data.bids_rigid_rigid[i];

            contact_active_pairs[i] = bool2(data_container->host_data.active_data[body.x], data_container->host_data.active_data[body.y]);

            contact_rotation[i] = data_container->host_data.rot_data[body.x];
            contact_rotation[i + num_contacts] = data_container->host_data.rot_data[body.y];

         }
      }
      solve_sliding = false;
      solve_spinning = false;
      count_shur_a_sliding = 0;

   }
   ~ChConstraintRigidRigid() {
   }
   void host_Project(
         int2 *ids,
         real3 *friction,
         real *cohesion,
         real *gamma);

   void Project(
         real* gamma);
   void Project_NoPar(
         real* gamma);
   void host_RHS(
         int2 *ids,
         real *correction,
         real alpha,
         bool2 * active,
         real3* norm,
         real3 *vel,
         real3 *omega,
         real3 *ptA,
         real3 *ptB,
         real4 *rot,
         real *rhs);
   void host_ComputeS(
         int2 *ids,
         real3 *mu,
         bool2 * active,
         real3* norm,
         real3 *vel,
         real3 *omega,
         real3 *ptA,
         real3 *ptB,
         real4 *rot,
         const real *rhs,
         real*b);
   void host_RHS_spinning(
         int2 *ids,
         bool2 * active,
         real3 *omega,
         real3 *norm,
         real4 * rot,
         real *rhs);

   void ComputeRHS();
   void UpdateRHS();
   void ComputeS(
         const custom_vector<real>& rhs,
         custom_vector<real3>& vel_data,
         custom_vector<real3>& omg_data,
         custom_vector<real>& b);
   void host_Jacobians(
         real3 *norm,
         real3 *ptA,
         real3 *ptB,
         int2 *ids,
         real4 *rot,
         real3* JUA,
         real3* JUB);

   void host_Jacobians_Rolling(
         real3 *norm,
         int2 *ids,
         real4 *rot,
         real3 *JTA,
         real3 *JTB,
         real3 *JSA,
         real3 *JSB,
         real3 *JRA,
         real3 *JRB);

   void ComputeJacobians();
   void UpdateJacobians();

   void host_shurA_normal(
         real *gamma,
         real3 *norm,
         real3 *JUA,
         real3 *JUB,
         real3 *updateV,
         real3 *updateO);

   void host_shurA_sliding(
         bool2* contact_active,
         real3* norm,
         real3 * ptA,
         real3 * ptB,
         real4 * rot,
         real * gamma,
         real3 * updateV,
         real3 * updateO);

   void host_shurA_spinning(
         bool2* contact_active,
         real3* norm,
         real3 * ptA,
         real3 * ptB,
         real4 * rot,
         real * gamma,
         real3 * updateV,
         real3 * updateO);

   void host_shurB_normal(
         const real & alpha,
         int2 * ids,
         bool2 *contact_active,
         real3 *norm,
         real4 *compliance,
         real *gamma,
         real3 *JUA,
         real3 *JUB,
         real3 *QXYZ,
         real3 *QUVW,
         real *AX);

   void host_shurB_sliding(
         const real & alpha,
         int2 * ids,
         bool2* contact_active,
         real3* norm,
         real4 * compliance,
         real * gamma,
         real3 * ptA,
         real3 * ptB,
         real4 * rot,
         real3 * QXYZ,
         real3 * QUVW,
         real * AX);

   void host_shurB_spinning(
         const real & alpha,
         int2 * ids,
         bool2* contact_active,
         real3* norm,
         real4 * compliance,
         real * gamma,
         real3 * ptA,
         real3 * ptB,
         real4 * rot,
         real3 * QXYZ,
         real3 * QUVW,
         real * AX);

   void host_Offsets(
         int2 *ids_contacts,
         int *Body);

   void host_Reduce_Shur(
         bool *active,
         real3 *QXYZ,
         real3 *QUVW,
         real *inv_mass,
         real3 *inv_inertia,
         real3 *updateQXYZ,
         real3 *updateQUVW,
         int *d_body_num,
         int *counter,
         int* reverse_offset);

   //void host_Diag(int2 *ids, bool *active, real *inv_mass, real3 *inv_inertia, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *diag);
   void Diag();

   void ShurA(
         real * x);
   void ShurB(
         real*x,
         real*output);

   void Build_N();
   void Build_D();

   bool solve_sliding;
   bool solve_spinning;
 protected:

   custom_vector<real3> JUA_rigid_rigid;custom_vector<real3> JUB_rigid_rigid;

   custom_vector<real4> comp_rigid_rigid;

   custom_vector<real3> vel_update, omg_update;

   custom_vector<int> body_num;

   custom_vector<int> update_number;custom_vector<int> update_offset;custom_vector<int2> update_offset_pairs;custom_vector<bool2> contact_active_pairs;custom_vector<real4> contact_rotation;custom_vector<
         int> update_offset_bodies;custom_vector<int> offset_counter;custom_vector<int> body_number;
   int count_shur_a_sliding;

   uint num_updates;

   real inv_hpa;
   real inv_hhpa;
};
}

#endif
