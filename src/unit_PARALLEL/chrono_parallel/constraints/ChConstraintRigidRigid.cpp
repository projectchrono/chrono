#include <algorithm>

#include "chrono_parallel/ChConfigParallel.h"
#include "ChConstraintRigidRigid.h"

using namespace chrono;
#define   _index_  index*offset

void chrono::Orthogonalize(real3 &Vx,
                           real3 &Vy,
                           real3 &Vz) {

//   if (U == R3(0)) {
//      U = R3(0, 1, 0);
//   } else {
//      U = normalize(U);
//   }
   real3 mVsingular = R3(0, 1, 0);
   Vz = cross(Vx, mVsingular);
   real mzlen = Vz.length();

   if (mzlen < real(0.0001)) {     // was near singularity? change singularity reference custom_vector!

      mVsingular = R3(1, 0, 0);

      Vz = cross(Vx, mVsingular);
      mzlen = Vz.length();
   }
   Vz = Vz / mzlen;
   Vy = cross(Vz, Vx);

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool Cone_generalized(real & gamma_n,
                      real & gamma_u,
                      real & gamma_v,
                      const real & mu) {

   real f_tang = sqrt(gamma_u * gamma_u + gamma_v * gamma_v);

   // inside upper cone? keep untouched!
   if (f_tang < (mu * gamma_n)) {
      return false;
   }

   // inside lower cone? reset  normal,u,v to zero!
   if ((f_tang) < -(1.0 / mu) * gamma_n || (fabs(gamma_n) < 10e-15)) {
      gamma_n = 0;
      gamma_u = 0;
      gamma_v = 0;
      return false;
   }

   // remaining case: project orthogonally to generator segment of upper cone

   gamma_n = (f_tang * mu + gamma_n) / (mu * mu + 1);
   real tproj_div_t = (gamma_n * mu) / f_tang;
   gamma_u *= tproj_div_t;
   gamma_v *= tproj_div_t;

   return true;
}

void Cone_single(real & gamma_n,
                 real & gamma_s,
                 const real & mu) {

   real f_tang = abs(gamma_s);

   // inside upper cone? keep untouched!
   if (f_tang < (mu * gamma_n)) {
      return;
   }

   // inside lower cone? reset  normal,u,v to zero!
   if ((f_tang) < -(1.0 / mu) * gamma_n || (fabs(gamma_n) < 10e-15)) {
      gamma_n = 0;
      gamma_s = 0;
      return;
   }

   // remaining case: project orthogonally to generator segment of upper cone

   gamma_n = (f_tang * mu + gamma_n) / (mu * mu + 1);
   real tproj_div_t = (gamma_n * mu) / f_tang;
   gamma_s *= tproj_div_t;

}

void ChConstraintRigidRigid::func_Project(int &index,
                                          int2 *ids,
                                          real3 *fric,
                                          real* cohesion,
                                          real *gam) {
   real3 gamma;
   gamma.x = gam[_index_ + 0];
   gamma.y = gam[_index_ + 1];
   gamma.z = gam[_index_ + 2];

   real coh = cohesion[index];
   gamma.x += coh;

   real mu = fric[index].x;
   if (mu == 0) {
      gamma.x = gamma.x < 0 ? 0 : gamma.x - coh;
      gamma.y = gamma.z = 0;

      gam[_index_ + 0] = gamma.x;
      gam[_index_ + 1] = gamma.y;
      gam[_index_ + 2] = gamma.z;

      return;
   }

   if (Cone_generalized(gamma.x, gamma.y, gamma.z, mu)) {
   }

   gam[_index_ + 0] = gamma.x - coh;
   gam[_index_ + 1] = gamma.y;
   gam[_index_ + 2] = gamma.z;

}
void ChConstraintRigidRigid::func_Project_rolling(int &index,
                                                  int2 *ids,
                                                  real3 *fric,
                                                  real *gam) {
   //real3 gamma_roll = R3(0);
   real rollingfriction = fric[index].y;
   real spinningfriction = fric[index].z;

//	if(rollingfriction||spinningfriction){
//		gam[index + number_of_contacts * 1] = 0;
//		gam[index + number_of_contacts * 2] = 0;
//	}

   real gamma_n = abs(gam[_index_ + 0]);
   real gamma_s = gam[_index_ + 3];
   real gamma_tu = gam[_index_ + 4];
   real gamma_tv = gam[_index_ + 5];

   if (spinningfriction == 0) {
      gamma_s = 0;

   } else {
      Cone_single(gamma_n, gamma_s, spinningfriction);
   }

   if (rollingfriction == 0) {
      gamma_tu = 0;
      gamma_tv = 0;
//		if (gamma_n < 0) {
//			gamma_n = 0;
//		}
   } else {
      Cone_generalized(gamma_n, gamma_tu, gamma_tv, rollingfriction);
   }
   //gam[index + number_of_contacts * 0] = gamma_n;
   gam[_index_ + 3] = gamma_s;
   gam[_index_ + 4] = gamma_tu;
   gam[_index_ + 5] = gamma_tv;

}

void ChConstraintRigidRigid::host_Project_single(int index,
                                                 int2 *ids,
                                                 real3 *friction,
                                                 real* cohesion,
                                                 real *gamma) {
   //always project normal
   if (solve_sliding) {
      func_Project(index, ids, friction, cohesion, gamma);
   } else {

      real gamma_x = gamma[_index_ + 0];
      int2 body_id = ids[index];
      real coh = (cohesion[body_id.x] + cohesion[body_id.y]) * .5;
      if (coh < 0) {
         coh = 0;
      }
      gamma_x += coh;

      gamma_x = gamma_x < 0 ? 0 : gamma_x - coh;

      gamma[_index_ + 0] = gamma_x;
      gamma[_index_ + 1] = 0;
      gamma[_index_ + 2] = 0;
   }
   if (solve_spinning) {

      func_Project_rolling(index, ids, friction, gamma);

   }

}

void ChConstraintRigidRigid::host_Project(int2 *ids,
                                          real3 *friction,
                                          real* cohesion,
                                          real *gamma) {
   //always project normal
   if (solve_sliding) {
#pragma omp parallel for
      for (int index = 0; index < num_contacts; index++) {
         func_Project(index, ids, friction, cohesion, gamma);
      }
   } else {
#pragma omp parallel for
      for (int index = 0; index < num_contacts; index++) {
         real gamma_x = gamma[_index_ + 0];
         int2 body_id = ids[index];
         real coh = (cohesion[body_id.x] + cohesion[body_id.y]) * .5;
         if (coh < 0) {
            coh = 0;
         }
         gamma_x += coh;

         gamma_x = gamma_x < 0 ? 0 : gamma_x - coh;

         gamma[_index_ + 0] = gamma_x;
         gamma[_index_ + 1] = 0;
         gamma[_index_ + 2] = 0;
      }
   }
   if (solve_spinning) {
#pragma omp parallel for
      for (int index = 0; index < num_contacts; index++) {
         func_Project_rolling(index, ids, friction, gamma);
      }
   }

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::Project(real* gamma) {
   host_Project(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.fric_rigid_rigid.data(), coh_rigid_rigid.data(), gamma);
}
void ChConstraintRigidRigid::Project_NoPar(real* gamma) {
   host_Project(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.fric_rigid_rigid.data(), coh_rigid_rigid.data(), gamma);
}
void ChConstraintRigidRigid::Project_Single(int index,
                                            real* gamma) {
   host_Project_single(index, data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.fric_rigid_rigid.data(), coh_rigid_rigid.data(), gamma);
}
void chrono::Compute_Jacobian(const real4& quat,
                              const real3& U,
                              const real3& V,
                              const real3& W,
                              const real3& point,
                              real3& T1,
                              real3& T2,
                              real3& T3) {
   real4 quaternion_conjugate = ~quat;

   real3 sbar = quatRotate(point, quaternion_conjugate);

   T1 = cross(quatRotate(U, quaternion_conjugate), sbar);
   T2 = cross(quatRotate(V, quaternion_conjugate), sbar);
   T3 = cross(quatRotate(W, quaternion_conjugate), sbar);

//  M33 X =  XMatrix(sbar);
//  M33 R = AMat(quat);
//
//  M33 T = R*X;
//  M33 C(U,V,W);
//  M33 Res = Transpose(MatTMult(C,T));
//
//  T1 = Res.U;
//  T2 = Res.V;
//  T3 = Res.W;

}

void chrono::Compute_Jacobian_Rolling(const real4& quat,
                                      const real3& U,
                                      const real3& V,
                                      const real3& W,
                                      real3& T1,
                                      real3& T2,
                                      real3& T3) {
   real4 quaternion_conjugate = ~quat;

   T1 = quatRotate(U, quaternion_conjugate);
   T2 = quatRotate(V, quaternion_conjugate);
   T3 = quatRotate(W, quaternion_conjugate);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void ChConstraintRigidRigid::host_RHS(int2 *ids,
                                      real *correction,
                                      real alpha,
                                      bool2 * active,
                                      real3* norm,
                                      real3 *vel,
                                      real3 *omega,
                                      real3 *ptA,
                                      real3 *ptB,
                                      real4 *rot,
                                      real *rhs) {

#pragma omp parallel for
   for (int index = 0; index < num_contacts; index++) {
      int2 bid = ids[index];
      bool2 isactive = active[index];

      real3 temp = R3(0);

      real3 U = norm[index], V, W;
      Orthogonalize(U, V, W);     //read 3 real

      if (isactive.x) {
         real3 omega_b1 = omega[bid.x];
         real3 vel_b1 = vel[bid.x];
         real3 T3, T4, T5;
         Compute_Jacobian(rot[index], U, V, W, ptA[index], T3, T4, T5);
         temp.x = dot(-U, vel_b1) + dot(T3, omega_b1);
         if (solve_sliding) {
            temp.y = dot(-V, vel_b1) + dot(T4, omega_b1);
            temp.z = dot(-W, vel_b1) + dot(T5, omega_b1);
         }
      }
      if (isactive.y) {
         real3 omega_b2 = omega[bid.y];
         real3 vel_b2 = vel[bid.y];
         real3 T6, T7, T8;
         Compute_Jacobian(rot[index + num_contacts], U, V, W, ptB[index], T6, T7, T8);
         temp.x += dot(U, vel_b2) + dot(-T6, omega_b2);
         if (solve_sliding) {
            temp.y += dot(V, vel_b2) + dot(-T7, omega_b2);
            temp.z += dot(W, vel_b2) + dot(-T8, omega_b2);
         }
      }
      real bi = 0;
      if (alpha) {
         bi = inv_hpa * correction[index];
      } else {
         bi = std::fmax(real(1.0) / step_size * correction[index], -contact_recovery_speed);
      }

      rhs[_index_ + 0] = -temp.x - bi;
      rhs[_index_ + 1] = -temp.y - 0;
      rhs[_index_ + 2] = -temp.z - 0;
   }
}

void ChConstraintRigidRigid::host_ComputeS(int2 *ids,
                                           real3 *mu,
                                           bool2 * active,
                                           real3* norm,
                                           real3 *vel,
                                           real3 *omega,
                                           real3 *ptA,
                                           real3 *ptB,
                                           real4 *rot,
                                           const real *rhs,
                                           real*b) {

#pragma omp parallel for
   for (int index = 0; index < num_contacts; index++) {

      int2 bid = ids[index];
      bool2 isactive = active[index];

      real3 temp = R3(0);

      real3 U = norm[index], V, W;
      Orthogonalize(U, V, W);     //read 3 real

      if (isactive.x) {
         real3 omega_b1 = omega[bid.x];
         real3 vel_b1 = vel[bid.x];
         real3 T3, T4, T5;
         Compute_Jacobian(rot[index], U, V, W, ptA[index], T3, T4, T5);
         temp.x = dot(-U, vel_b1) + dot(T3, omega_b1);
         temp.y = dot(-V, vel_b1) + dot(T4, omega_b1);
         temp.z = dot(-W, vel_b1) + dot(T5, omega_b1);
      }
      if (isactive.y) {
         real3 omega_b2 = omega[bid.y];
         real3 vel_b2 = vel[bid.y];
         real3 T6, T7, T8;
         Compute_Jacobian(rot[index + num_contacts], U, V, W, ptB[index], T6, T7, T8);
         temp.x += dot(U, vel_b2) + dot(-T6, omega_b2);
         temp.y += dot(V, vel_b2) + dot(-T7, omega_b2);
         temp.z += dot(W, vel_b2) + dot(-T8, omega_b2);
      }
      real3 f_a = mu[bid.x];
      real3 f_b = mu[bid.y];

      real fric = (f_a.x == 0 || f_b.x == 0) ? 0 : (f_a.x + f_b.x) * .5;
      real s = sqrt(temp.y * temp.y + temp.z * temp.z) * fric;
      b[_index_ + 0] = rhs[_index_ + 0] - s;
      //cout<<"here: "<<s<<endl;

   }
}

void ChConstraintRigidRigid::host_RHS_spinning(int2 *ids,
                                               bool2 * active,
                                               real3 *omega,
                                               real3 *norm,
                                               real4 * rot,
                                               real *rhs) {

#pragma omp parallel for
   for (int index = 0; index < num_contacts; index++) {
      int2 bid = ids[index];
      bool2 isactive = active[index];
      real3 temp = R3(0);

      real3 U = norm[index], V, W;
      Orthogonalize(U, V, W);     //read 3 real

      if (isactive.x) {

         real4 quat = rot[index];

         real3 TA, TB, TC;
         Compute_Jacobian_Rolling(quat, U, V, W, TA, TB, TC);

         real3 omega_b1 = omega[bid.x];
         temp.x = dot(-TA, omega_b1);
         temp.y = dot(-TB, omega_b1);
         temp.z = dot(-TC, omega_b1);
      }
      if (isactive.y) {

         real4 quat = rot[index + num_contacts];
         real3 TA, TB, TC;
         Compute_Jacobian_Rolling(quat, U, V, W, TA, TB, TC);

         real3 omega_b2 = omega[bid.y];
         temp.x += dot(TA, omega_b2);
         temp.y += dot(TB, omega_b2);
         temp.z += dot(TC, omega_b2);
      }

      rhs[_index_ + 3] = -temp.x;
      rhs[_index_ + 4] = -temp.y;
      rhs[_index_ + 5] = -temp.z;
   }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::ComputeRHS() {
   data_container->system_timer.start("ChLcpSolverParallel_RHS");
   comp_rigid_rigid.resize(num_contacts);
   coh_rigid_rigid.resize(num_contacts);
   data_container->host_data.fric_rigid_rigid.resize(num_contacts);

#pragma omp parallel for
   for (int i = 0; i < num_contacts; i++) {
      uint b1 = data_container->host_data.bids_rigid_rigid[i].x;
      uint b2 = data_container->host_data.bids_rigid_rigid[i].y;

      real4 compb1 = data_container->host_data.compliance_data[b1];
      real4 compb2 = data_container->host_data.compliance_data[b2];
      real4 compab = R4(0.0);

      compab.x = (compb1.x == 0 || compb2.x == 0) ? 0 : (compb1.x + compb2.x) * .5;
      compab.y = (compb1.y == 0 || compb2.y == 0) ? 0 : (compb1.y + compb2.y) * .5;
      compab.z = (compb1.z == 0 || compb2.z == 0) ? 0 : (compb1.z + compb2.z) * .5;
      compab.w = (compb1.w == 0 || compb2.w == 0) ? 0 : (compb1.w + compb2.w) * .5;
      comp_rigid_rigid[i] = inv_hhpa * compab;

      real coh = std::max((data_container->host_data.cohesion_data[b1] + data_container->host_data.cohesion_data[b2]) * .5, 0.0);
      coh_rigid_rigid[i] = coh;
      real3 f_a = data_container->host_data.fric_data[b1];
      real3 f_b = data_container->host_data.fric_data[b2];
      real3 mu;

      mu.x = (f_a.x == 0 || f_b.x == 0) ? 0 : (f_a.x + f_b.x) * .5;
      mu.y = (f_a.y == 0 || f_b.y == 0) ? 0 : (f_a.y + f_b.y) * .5;
      mu.z = (f_a.z == 0 || f_b.z == 0) ? 0 : (f_a.z + f_b.z) * .5;

      data_container->host_data.fric_rigid_rigid[i] = mu;

   }
   host_RHS(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.dpth_rigid_rigid.data(), data_container->settings.solver.alpha, contact_active_pairs.data(),
            data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.vel_data.data(), data_container->host_data.omg_data.data(),
            data_container->host_data.cpta_rigid_rigid.data(), data_container->host_data.cptb_rigid_rigid.data(), contact_rotation.data(),
            data_container->host_data.rhs_data.data());
   if (solve_spinning) {
      host_RHS_spinning(data_container->host_data.bids_rigid_rigid.data(), contact_active_pairs.data(), data_container->host_data.omg_data.data(),
                        data_container->host_data.norm_rigid_rigid.data(), contact_rotation.data(), data_container->host_data.rhs_data.data());
   }
   data_container->system_timer.stop("ChLcpSolverParallel_RHS");
}

void ChConstraintRigidRigid::UpdateRHS() {
   host_RHS(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.dpth_rigid_rigid.data(), data_container->settings.solver.alpha, contact_active_pairs.data(),
            data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.vel_new_data.data(), data_container->host_data.omg_new_data.data(),
            data_container->host_data.cpta_rigid_rigid.data(), data_container->host_data.cptb_rigid_rigid.data(), contact_rotation.data(),
            data_container->host_data.rhs_data.data());
   if (solve_spinning) {
      host_RHS_spinning(data_container->host_data.bids_rigid_rigid.data(), contact_active_pairs.data(), data_container->host_data.omg_new_data.data(),
                        data_container->host_data.norm_rigid_rigid.data(), contact_rotation.data(), data_container->host_data.rhs_data.data());
   }
}

void ChConstraintRigidRigid::ComputeS(const custom_vector<real>& rhs,
                                      custom_vector<real3>& vel_data,
                                      custom_vector<real3>& omg_data,
                                      custom_vector<real>& b) {
   if (solve_sliding) {
      host_ComputeS(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.fric_data.data(), contact_active_pairs.data(),
                    data_container->host_data.norm_rigid_rigid.data(), vel_data.data(), omg_data.data(), data_container->host_data.cpta_rigid_rigid.data(),
                    data_container->host_data.cptb_rigid_rigid.data(), contact_rotation.data(), rhs.data(), b.data());
   }

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::host_Jacobians(real3* norm,
                                            real3* ptA,
                                            real3* ptB,
                                            int2* ids,
                                            real4* rot,
                                            real3* JUA,
                                            real3* JUB) {
#pragma omp parallel for
   for (int index = 0; index < num_contacts; index++) {
      real3 U = norm[index], V, W;
      Orthogonalize(U, V, W);

      int2 body_id = ids[index];

      real3 T3, T4, T5, T6, T7, T8;
      Compute_Jacobian(rot[index], U, V, W, ptA[index], T3, T4, T5);
      Compute_Jacobian(rot[index + num_contacts], U, V, W, ptB[index], T6, T7, T8);
      T6 = -T6;
      T7 = -T7;
      T8 = -T8;

      JUA[index] = T3;
//		JVA[index] = T4;
//		JWA[index] = T5;

      JUB[index] = T6;
//		JVB[index] = T7;
//		JWB[index] = T8;
   }
}

void ChConstraintRigidRigid::host_Jacobians_Rolling(real3* norm,
                                                    int2* ids,
                                                    real4* rot,
                                                    real3 *JTA,
                                                    real3 *JTB,
                                                    real3 *JSA,
                                                    real3 *JSB,
                                                    real3 *JRA,
                                                    real3 *JRB) {
//#pragma omp parallel for
//	for (int index = 0; index < num_contacts; index++) {
//		real3 U = norm[index], V, W;
//		Orthogonalize(U, V, W);
//
//		int2 body_id = ids[index];
//		real3 T3, T4, T5, T6, T7, T8;
//
//		Compute_Jacobian_Rolling(rot[body_id.x], U, V, W, T3, T4, T5);
//		T3 = -T3;
//		T4 = -T4;
//		T5 = -T5;
//
//		Compute_Jacobian_Rolling(rot[body_id.y], U, V, W, T6, T7, T8);
//
//		JTA[index] = T3;
//		JSA[index] = T4;
//		JRA[index] = T5;
//
//		JTB[index] = T6;
//		JSB[index] = T7;
//		JRB[index] = T8;
//	}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::ComputeJacobians() {
   JUA_rigid_rigid.resize(num_contacts);
   JUB_rigid_rigid.resize(num_contacts);

   host_Jacobians(data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.cpta_rigid_rigid.data(), data_container->host_data.cptb_rigid_rigid.data(),
                  data_container->host_data.bids_rigid_rigid.data(), contact_rotation.data(), JUA_rigid_rigid.data(), JUB_rigid_rigid.data());

}

void ChConstraintRigidRigid::UpdateJacobians() {

   JUA_rigid_rigid.resize(num_contacts);
   JUB_rigid_rigid.resize(num_contacts);

   host_Jacobians(data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.cpta_rigid_rigid.data(), data_container->host_data.cptb_rigid_rigid.data(),
                  data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.rot_new_data.data(), JUA_rigid_rigid.data(), JUB_rigid_rigid.data());

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::host_shurA_normal(real * gamma,
                                               real3* norm,
                                               real3 * JUA,
                                               real3 * JUB,
                                               real3 * updateV,
                                               real3 * updateO) {
#pragma omp parallel for
   for (int index = 0; index < num_contacts; index++) {
      real gam = gamma[index * 6];
      real3 U = norm[index];
      updateV[index] = -U * gam;
      //updateV[index + num_contacts] = U * gam;
      updateO[index] = JUA[index] * gam;
      updateO[index + num_contacts] = JUB[index] * gam;

   }
}

void ChConstraintRigidRigid::host_shurA_sliding(bool2* contact_active,
                                                real3* norm,
                                                real3 * ptA,
                                                real3 * ptB,
                                                real4 * rot,
                                                real * gamma,
                                                real3 * updateV,
                                                real3 * updateO) {
#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int index = 0; index < num_contacts; index++) {
#ifdef ENABLE_SSE
      real3 gam(_mm_loadu_ps(&gamma[_index_]));
#else
      real3 gam(gamma[_index_ + 0], gamma[_index_ + 1], gamma[_index_ + 2]);
#endif
      real3 U = norm[index], V, W;
      Orthogonalize(U, V, W);
      bool2 active = contact_active[index];

      updateV[index] = -U * gam.x - V * gam.y - W * gam.z;
      if (active.x != 0) {
         real3 T3, T4, T5;
         Compute_Jacobian(rot[index], U, V, W, ptA[index], T3, T4, T5);
         updateO[index] = T3 * gam.x + T4 * gam.y + T5 * gam.z;
      }
      if (active.y != 0) {
         real3 T6, T7, T8;
         Compute_Jacobian(rot[index + num_contacts], U, V, W, ptB[index], T6, T7, T8);
         //updateV[index + num_contacts] = U * gam.x + V * gam.y + W * gam.z;
         updateO[index + num_contacts] = -T6 * gam.x - T7 * gam.y - T8 * gam.z;
      }
   }
}

void ChConstraintRigidRigid::host_shurA_spinning(bool2* contact_active,
                                                 real3* norm,
                                                 real3 * ptA,
                                                 real3 * ptB,
                                                 real4 * rot,
                                                 real * gamma,
                                                 real3 * updateV,
                                                 real3 * updateO) {
#pragma omp parallel for
   for (int index = 0; index < num_contacts; index++) {
#ifdef ENABLE_SSE
      real3 gam(_mm_loadu_ps(&gamma[_index_]));
#else
      real3 gam(gamma[_index_ + 0], gamma[_index_ + 1], gamma[_index_ + 2]);
#endif
      real3 gam_roll(gamma[_index_ + 3], gamma[_index_ + 4], gamma[_index_ + 5]);

      real3 U = norm[index], V, W;
      Orthogonalize(U, V, W);
      bool2 active = contact_active[index];
      updateV[index] = -U * gam.x - V * gam.y - W * gam.z;

      if (active.x != 0) {
         real4 quat = rot[index];
         real3 T3, T4, T5, TA, TB, TC;
         Compute_Jacobian(quat, U, V, W, ptA[index], T3, T4, T5);
         Compute_Jacobian_Rolling(quat, U, V, W, TA, TB, TC);

         updateO[index] = T3 * gam.x + T4 * gam.y + T5 * gam.z - TA * gam_roll.x - TB * gam_roll.y - TC * gam_roll.z;
      }
      if (active.y != 0) {
         real4 quat = rot[index + num_contacts];
         real3 T6, T7, T8, TA, TB, TC;
         Compute_Jacobian(quat, U, V, W, ptB[index], T6, T7, T8);
         Compute_Jacobian_Rolling(quat, U, V, W, TA, TB, TC);
         //updateV[index + num_contacts] = U * gam.x + V * gam.y + W * gam.z;
         updateO[index + num_contacts] = -T6 * gam.x - T7 * gam.y - T8 * gam.z + TA * gam_roll.x + TB * gam_roll.y + TC * gam_roll.z;
      }
   }
}

void ChConstraintRigidRigid::host_shurB_normal(const real & alpha,
                                               int2 * ids,
                                               bool2* contact_active,
                                               real3* norm,
                                               real4 * compliance,
                                               real * gamma,
                                               real3 * JUA,
                                               real3 * JUB,
                                               real3 * QXYZ,
                                               real3 * QUVW,
                                               real * AX) {

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int index = 0; index < num_contacts; index++) {
      real temp = 0;

      int2 id_ = ids[index];
      real3 XYZ, UVW;
      real3 U = norm[index];
      bool2 active = contact_active[index];
      if (active.x != 0) {
         XYZ = QXYZ[id_.x];
         UVW = QUVW[id_.x];
         temp = dot(XYZ, -U) + dot(UVW, JUA[index]);
      }
      if (active.y != 0) {
         XYZ = QXYZ[id_.y];
         UVW = QUVW[id_.y];
         temp += dot(XYZ, U) + dot(UVW, JUB[index]);
      }

      real4 comp = compliance[index];
      //temp += (alpha) ? gamma[_index_] * comp.x : 0;
      AX[_index_ + 0] = temp;
//		_mm_storeu_ps(&AX[_index_ + 1], _mm_setzero_ps());
//		AX[_index_ + 5] = 0;

   }
}
void ChConstraintRigidRigid::host_shurB_sliding(const real & alpha,
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
                                                real * AX) {

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int index = 0; index < num_contacts; index++) {
      real3 temp = R3(0);

      int2 bid = ids[index];
      real3 XYZ, UVW;
      real3 U = norm[index], V, W;
      Orthogonalize(U, V, W);

      bool2 active = contact_active[index];
      if (active.x != 0) {
         XYZ = QXYZ[bid.x];
         UVW = QUVW[bid.x];
         real3 T3, T4, T5;
         Compute_Jacobian(rot[index], U, V, W, ptA[index], T3, T4, T5);
         temp.x = dot(XYZ, -U) + dot(UVW, T3);
         temp.y = dot(XYZ, -V) + dot(UVW, T4);
         temp.z = dot(XYZ, -W) + dot(UVW, T5);
      }

      if (active.y != 0) {
         XYZ = QXYZ[bid.y];
         UVW = QUVW[bid.y];
         real3 T6, T7, T8;
         Compute_Jacobian(rot[index + num_contacts], U, V, W, ptB[index], T6, T7, T8);
         temp.x += dot(XYZ, U) + dot(UVW, -T6);
         temp.y += dot(XYZ, V) + dot(UVW, -T7);
         temp.z += dot(XYZ, W) + dot(UVW, -T8);
      }

      real4 comp = compliance[index];
#ifdef ENABLE_SSE
      real3 gam(_mm_loadu_ps(&gamma[_index_]));
#else
      real3 gam(gamma[_index_ + 0], gamma[_index_ + 1], gamma[_index_ + 2]);
#endif
      //temp += (alpha) ? gam * real3(comp.x,comp.y,comp.y) : 0;

      AX[_index_ + 0] = temp.x;
      AX[_index_ + 1] = temp.y;
      AX[_index_ + 2] = temp.z;
   }
}
void ChConstraintRigidRigid::host_shurB_spinning(const real & alpha,
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
                                                 real * AX) {

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int index = 0; index < num_contacts; index++) {
      real3 temp = R3(0);
      real3 temp_roll = R3(0);

      int2 bid = ids[index];

      real3 U = norm[index], V, W;
      Orthogonalize(U, V, W);
      bool2 active = contact_active[index];
      if (active.x != 0) {
         real3 XYZ = QXYZ[bid.x];
         real3 UVW = QUVW[bid.x];

         real4 quat = rot[index];
         real3 TA, TB, TC;
         Compute_Jacobian_Rolling(quat, U, V, W, TA, TB, TC);

         real3 T3, T4, T5;
         Compute_Jacobian(quat, U, V, W, ptA[index], T3, T4, T5);
         temp.x = dot(XYZ, -U) + dot(UVW, T3);
         temp.y = dot(XYZ, -V) + dot(UVW, T4);
         temp.z = dot(XYZ, -W) + dot(UVW, T5);

         temp_roll.x = dot(UVW, -TA);
         temp_roll.y = dot(UVW, -TB);
         temp_roll.z = dot(UVW, -TC);

      }
      if (active.y != 0) {
         real3 XYZ = QXYZ[bid.y];
         real3 UVW = QUVW[bid.y];

         real4 quat = rot[index + num_contacts];
         real3 TA, TB, TC;
         Compute_Jacobian_Rolling(quat, U, V, W, TA, TB, TC);

         real3 T6, T7, T8;
         Compute_Jacobian(quat, U, V, W, ptB[index], T6, T7, T8);
         temp.x += dot(XYZ, U) + dot(UVW, -T6);
         temp.y += dot(XYZ, V) + dot(UVW, -T7);
         temp.z += dot(XYZ, W) + dot(UVW, -T8);

         temp_roll.x += dot(UVW, TA);
         temp_roll.y += dot(UVW, TB);
         temp_roll.z += dot(UVW, TC);

      }
      //real4 comp = compliance[index];
      //real3 gam(_mm_loadu_ps(&gamma[_index_]));
      //real3 gam_roll(_mm_loadu_ps(&gamma[_index_ + 3]));

      //temp += (alpha) ? gam * real3(comp.x,comp.y,comp.y) : 0;
      //temp_roll += (alpha) ? gam_roll * real3(comp.w,comp.z,comp.z) : 0;

      AX[_index_ + 0] = temp.x;
      AX[_index_ + 1] = temp.y;
      AX[_index_ + 2] = temp.z;

      AX[_index_ + 3] = temp_roll.x;
      AX[_index_ + 4] = temp_roll.y;
      AX[_index_ + 5] = temp_roll.z;

   }
}

void ChConstraintRigidRigid::host_Reduce_Shur(bool* active,
                                              real3* QXYZ,
                                              real3* QUVW,
                                              real * inv_mass,
                                              real3 * inv_inertia,
                                              real3* updateQXYZ,
                                              real3* updateQUVW,
                                              int* d_body_num,
                                              int* counter,
                                              int* reverse_offset) {
#pragma omp parallel for
   for (int index = 0; index < num_updates; index++) {

      int start = (index == 0) ? 0 : counter[index - 1], end = counter[index];
      int id = d_body_num[end - 1], j;

      if (active[id] != 0) {

         real3 mUpdateV = R3(0);
         real3 mUpdateO = R3(0);

         for (j = 0; j < end - start; j++) {
            int contact_num = reverse_offset[j + start];
            if (contact_num >= num_contacts) {
               mUpdateV += -updateQXYZ[contact_num - num_contacts];
            } else {
               mUpdateV += updateQXYZ[contact_num];
            }
            mUpdateO += updateQUVW[contact_num];
         }
         QXYZ[id] = mUpdateV * inv_mass[id];
         QUVW[id] = mUpdateO * inv_inertia[id];
      }
   }
}

void ChConstraintRigidRigid::host_Offsets(int2* ids,
                                          int* Body) {
#pragma omp parallel for
   for (int index = 0; index < num_contacts; index++) {
      if (index < num_contacts) {
         int2 temp_id = ids[index];
         Body[index] = temp_id.x;
         Body[index + num_contacts] = temp_id.y;
      }
   }
}

void ChConstraintRigidRigid::ShurA(real* x) {

   if (solve_spinning) {

      data_container->system_timer.SetMemory("ChConstraintRigidRigid_shurA_spinning", (10 * 4 + 2) * 4 * num_contacts);
      data_container->system_timer.start("ChConstraintRigidRigid_shurA_spinning");

      host_shurA_spinning(contact_active_pairs.data(), data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.cpta_rigid_rigid.data(),
                          data_container->host_data.cptb_rigid_rigid.data(), contact_rotation.data(), x, vel_update.data(), omg_update.data());

      data_container->system_timer.stop("ChConstraintRigidRigid_shurA_spinning");

   } else if (solve_sliding) {

      data_container->system_timer.SetMemory("ChConstraintRigidRigid_shurA_sliding", (9 * 4 + 2) * 4 * num_contacts);
      data_container->system_timer.start("ChConstraintRigidRigid_shurA_sliding");

      host_shurA_sliding(contact_active_pairs.data(), data_container->host_data.norm_rigid_rigid.data(), data_container->host_data.cpta_rigid_rigid.data(),
                         data_container->host_data.cptb_rigid_rigid.data(), contact_rotation.data(), x, vel_update.data(), omg_update.data());

      data_container->system_timer.stop("ChConstraintRigidRigid_shurA_sliding");

   } else {

      data_container->system_timer.SetMemory("ChConstraintRigidRigid_shurA_normal", (6 * 4 * 4 + 1 * 4) * num_contacts);
      data_container->system_timer.start("ChConstraintRigidRigid_shurA_normal");

      host_shurA_normal(x, data_container->host_data.norm_rigid_rigid.data(), JUA_rigid_rigid.data(), JUB_rigid_rigid.data(), vel_update.data(), omg_update.data());

      data_container->system_timer.stop("ChConstraintRigidRigid_shurA_normal");

   }

   data_container->system_timer.start("ChConstraintRigidRigid_shurA_reduce");

   host_Reduce_Shur(data_container->host_data.active_data.data(), data_container->host_data.QXYZ_data.data(), data_container->host_data.QUVW_data.data(),
                    data_container->host_data.mass_data.data(), data_container->host_data.inr_data.data(), vel_update.data(), omg_update.data(), body_number.data(),
                    offset_counter.data(), update_offset_bodies.data());

   data_container->system_timer.stop("ChConstraintRigidRigid_shurA_reduce");

}
void ChConstraintRigidRigid::ShurB(real*x,
                                   real* output) {

   if (solve_spinning) {
      data_container->system_timer.start("ChConstraintRigidRigid_shurB_spinning");
      host_shurB_spinning(data_container->settings.solver.alpha, data_container->host_data.bids_rigid_rigid.data(), contact_active_pairs.data(), data_container->host_data.norm_rigid_rigid.data(),
                          comp_rigid_rigid.data(), x, data_container->host_data.cpta_rigid_rigid.data(), data_container->host_data.cptb_rigid_rigid.data(), contact_rotation.data(),
                          data_container->host_data.QXYZ_data.data(), data_container->host_data.QUVW_data.data(), output);
      data_container->system_timer.stop("ChConstraintRigidRigid_shurB_spinning");
   } else if (solve_sliding) {
      data_container->system_timer.start("ChConstraintRigidRigid_shurB_sliding");
      host_shurB_sliding(data_container->settings.solver.alpha, data_container->host_data.bids_rigid_rigid.data(), contact_active_pairs.data(), data_container->host_data.norm_rigid_rigid.data(),
                         comp_rigid_rigid.data(), x, data_container->host_data.cpta_rigid_rigid.data(), data_container->host_data.cptb_rigid_rigid.data(), contact_rotation.data(),
                         data_container->host_data.QXYZ_data.data(), data_container->host_data.QUVW_data.data(), output);
      data_container->system_timer.stop("ChConstraintRigidRigid_shurB_sliding");

   } else {
      data_container->system_timer.start("ChConstraintRigidRigid_shurB_normal");
      host_shurB_normal(data_container->settings.solver.alpha, data_container->host_data.bids_rigid_rigid.data(), contact_active_pairs.data(), data_container->host_data.norm_rigid_rigid.data(),
                        comp_rigid_rigid.data(), x, JUA_rigid_rigid.data(), JUB_rigid_rigid.data(), data_container->host_data.QXYZ_data.data(),
                        data_container->host_data.QUVW_data.data(), output);
      data_container->system_timer.stop("ChConstraintRigidRigid_shurB_normal");
   }
}

void ChConstraintRigidRigid::Build_D(SOLVERMODE solver_mode) {
   real3 * norm = data_container->host_data.norm_rigid_rigid.data();
   real3 * ptA = data_container->host_data.cpta_rigid_rigid.data();
   real3 *ptB = data_container->host_data.cptb_rigid_rigid.data();
   int2 * ids = data_container->host_data.bids_rigid_rigid.data();
   real4* rot = contact_rotation.data();



   for (int index = 0; index < num_contacts; index++) {
      real3 U = norm[index], V, W;
      Orthogonalize(U, V, W);

      int2 body_id = ids[index];

      int index_mult = index * 3;

      /////
      data_container->host_data.D.insert(body_id.x * 6 + 0, index_mult + 0, -U.x);
      data_container->host_data.D.insert(body_id.x * 6 + 1, index_mult + 0, -U.y);
      data_container->host_data.D.insert(body_id.x * 6 + 2, index_mult + 0, -U.z);

      data_container->host_data.D.insert(body_id.x * 6 + 0, index_mult + 1, -V.x);
      data_container->host_data.D.insert(body_id.x * 6 + 1, index_mult + 1, -V.y);
      data_container->host_data.D.insert(body_id.x * 6 + 2, index_mult + 1, -V.z);

      data_container->host_data.D.insert(body_id.x * 6 + 0, index_mult + 2, -W.x);
      data_container->host_data.D.insert(body_id.x * 6 + 1, index_mult + 2, -W.y);
      data_container->host_data.D.insert(body_id.x * 6 + 2, index_mult + 2, -W.z);

      data_container->host_data.D.insert(body_id.y * 6 + 0, index_mult + 0, U.x);
      data_container->host_data.D.insert(body_id.y * 6 + 1, index_mult + 0, U.y);
      data_container->host_data.D.insert(body_id.y * 6 + 2, index_mult + 0, U.z);

      data_container->host_data.D.insert(body_id.y * 6 + 0, index_mult + 1, V.x);
      data_container->host_data.D.insert(body_id.y * 6 + 1, index_mult + 1, V.y);
      data_container->host_data.D.insert(body_id.y * 6 + 2, index_mult + 1, V.z);

      data_container->host_data.D.insert(body_id.y * 6 + 0, index_mult + 2, W.x);
      data_container->host_data.D.insert(body_id.y * 6 + 1, index_mult + 2, W.y);
      data_container->host_data.D.insert(body_id.y * 6 + 2, index_mult + 2, W.z);

      if (solver_mode == SLIDING || solver_mode == SPINNING) {

         real3 T3, T4, T5, T6, T7, T8;
         Compute_Jacobian(rot[index], U, V, W, ptA[index], T3, T4, T5);
         Compute_Jacobian(rot[index + num_contacts], U, V, W, ptB[index], T6, T7, T8);

         data_container->host_data.D.insert(body_id.x * 6 + 3, index_mult + 0, T3.x);
         data_container->host_data.D.insert(body_id.x * 6 + 4, index_mult + 0, T3.y);
         data_container->host_data.D.insert(body_id.x * 6 + 5, index_mult + 0, T3.z);

         data_container->host_data.D.insert(body_id.x * 6 + 3, index_mult + 1, T4.x);
         data_container->host_data.D.insert(body_id.x * 6 + 4, index_mult + 1, T4.y);
         data_container->host_data.D.insert(body_id.x * 6 + 5, index_mult + 1, T4.z);

         data_container->host_data.D.insert(body_id.x * 6 + 3, index_mult + 2, T5.x);
         data_container->host_data.D.insert(body_id.x * 6 + 4, index_mult + 2, T5.y);
         data_container->host_data.D.insert(body_id.x * 6 + 5, index_mult + 2, T5.z);

         /////

         data_container->host_data.D.insert(body_id.y * 6 + 3, index_mult + 0, -T6.x);
         data_container->host_data.D.insert(body_id.y * 6 + 4, index_mult + 0, -T6.y);
         data_container->host_data.D.insert(body_id.y * 6 + 5, index_mult + 0, -T6.z);

         data_container->host_data.D.insert(body_id.y * 6 + 3, index_mult + 1, -T7.x);
         data_container->host_data.D.insert(body_id.y * 6 + 4, index_mult + 1, -T7.y);
         data_container->host_data.D.insert(body_id.y * 6 + 5, index_mult + 1, -T7.z);

         data_container->host_data.D.insert(body_id.y * 6 + 3, index_mult + 2, -T8.x);
         data_container->host_data.D.insert(body_id.y * 6 + 4, index_mult + 2, -T8.y);
         data_container->host_data.D.insert(body_id.y * 6 + 5, index_mult + 2, -T8.z);
      }
      if (solver_mode == SPINNING) {
         real3 TA, TB, TC;
         Compute_Jacobian_Rolling(rot[index], U, V, W, TA, TB, TC);

         real3 TD, TE, TF;
         Compute_Jacobian_Rolling(rot[index + num_contacts], U, V, W, TD, TE, TF);

         data_container->host_data.D.insert(body_id.x * 6 + 3, index * 6 + 3, -TA.x);
         data_container->host_data.D.insert(body_id.x * 6 + 4, index * 6 + 3, -TA.y);
         data_container->host_data.D.insert(body_id.x * 6 + 5, index * 6 + 3, -TA.z);

         data_container->host_data.D.insert(body_id.x * 6 + 3, index * 6 + 4, -TB.x);
         data_container->host_data.D.insert(body_id.x * 6 + 4, index * 6 + 4, -TB.y);
         data_container->host_data.D.insert(body_id.x * 6 + 5, index * 6 + 4, -TB.z);

         data_container->host_data.D.insert(body_id.x * 6 + 3, index * 6 + 5, -TC.x);
         data_container->host_data.D.insert(body_id.x * 6 + 4, index * 6 + 5, -TC.y);
         data_container->host_data.D.insert(body_id.x * 6 + 5, index * 6 + 5, -TC.z);
         /////
         data_container->host_data.D.insert(body_id.y * 6 + 3, index * 6 + 3, TD.x);
         data_container->host_data.D.insert(body_id.y * 6 + 4, index * 6 + 3, TD.y);
         data_container->host_data.D.insert(body_id.y * 6 + 5, index * 6 + 3, TD.z);

         data_container->host_data.D.insert(body_id.y * 6 + 3, index * 6 + 4, TE.x);
         data_container->host_data.D.insert(body_id.y * 6 + 4, index * 6 + 4, TE.y);
         data_container->host_data.D.insert(body_id.y * 6 + 5, index * 6 + 4, TE.z);

         data_container->host_data.D.insert(body_id.y * 6 + 3, index * 6 + 5, TF.x);
         data_container->host_data.D.insert(body_id.y * 6 + 4, index * 6 + 5, TF.y);
         data_container->host_data.D.insert(body_id.y * 6 + 5, index * 6 + 5, TF.z);
      }
   }
}
