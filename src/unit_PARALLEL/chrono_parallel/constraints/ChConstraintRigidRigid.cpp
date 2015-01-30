#include <algorithm>

#include "chrono_parallel/ChConfigParallel.h"
#include "chrono_parallel/constraints/ChConstraintRigidRigid.h"

using namespace chrono;

void chrono::Orthogonalize(real3& Vx, real3& Vy, real3& Vz) {
  real3 mVsingular = R3(0, 1, 0);
  Vz = cross(Vx, mVsingular);
  real mzlen = Vz.length();
  // was near singularity? change singularity reference vector!
  if (mzlen < real(0.0001)) {
    mVsingular = R3(1, 0, 0);
    Vz = cross(Vx, mVsingular);
    mzlen = Vz.length();
  }
  Vz = Vz / mzlen;
  Vy = cross(Vz, Vx);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool Cone_generalized(real& gamma_n, real& gamma_u, real& gamma_v, const real& mu) {
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

void Cone_single(real& gamma_n, real& gamma_s, const real& mu) {
  real f_tang = fabs(gamma_s);

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

void ChConstraintRigidRigid::func_Project_normal(int index, const int2* ids, const real* cohesion, real* gamma) {

  real gamma_x = gamma[index * 1 + 0];
  int2 body_id = ids[index];
  real coh = cohesion[index];

  gamma_x += coh;
  gamma_x = gamma_x < 0 ? 0 : gamma_x - coh;
  gamma[index * 1 + 0] = gamma_x;
  if (data_container->settings.solver.solver_mode == SLIDING) {
    gamma[data_container->num_contacts + index * 2 + 0] = 0;
    gamma[data_container->num_contacts + index * 2 + 1] = 0;
  }
  if (data_container->settings.solver.solver_mode == SPINNING) {
    gamma[3 * data_container->num_contacts + index * 3 + 0] = 0;
    gamma[3 * data_container->num_contacts + index * 3 + 1] = 0;
    gamma[3 * data_container->num_contacts + index * 3 + 2] = 0;
  }
}

void ChConstraintRigidRigid::func_Project_sliding(int index, const int2* ids, const real3* fric, const real* cohesion, real* gam) {
  real3 gamma;
  gamma.x = gam[index * 1 + 0];
  gamma.y = gam[data_container->num_contacts + index * 2 + 0];
  gamma.z = gam[data_container->num_contacts + index * 2 + 1];

  if (data_container->settings.solver.solver_mode == SPINNING) {
    gamma[3 * data_container->num_contacts + index * 3 + 0] = 0;
    gamma[3 * data_container->num_contacts + index * 3 + 1] = 0;
    gamma[3 * data_container->num_contacts + index * 3 + 2] = 0;
  }

  real coh = cohesion[index];
  gamma.x += coh;

  real mu = fric[index].x;
  if (mu == 0) {
    gamma.x = gamma.x < 0 ? 0 : gamma.x - coh;
    gamma.y = gamma.z = 0;

    gam[index * 1 + 0] = gamma.x;
    gam[data_container->num_contacts + index * 2 + 0] = gamma.y;
    gam[data_container->num_contacts + index * 2 + 1] = gamma.z;

    return;
  }

  if (Cone_generalized(gamma.x, gamma.y, gamma.z, mu)) {
  }

  gam[index * 1 + 0] = gamma.x - coh;
  gam[data_container->num_contacts + index * 2 + 0] = gamma.y;
  gam[data_container->num_contacts + index * 2 + 1] = gamma.z;
}
void ChConstraintRigidRigid::func_Project_spinning(int index, const int2* ids, const real3* fric, real* gam) {
  // real3 gamma_roll = R3(0);
  real rollingfriction = fric[index].y;
  real spinningfriction = fric[index].z;

  //	if(rollingfriction||spinningfriction){
  //		gam[index + number_of_contacts * 1] = 0;
  //		gam[index + number_of_contacts * 2] = 0;
  //	}

  real gamma_n = fabs(gam[index * 1 + 0]);
  real gamma_s  = gam[3 * data_container->num_contacts + index * 3 + 0];
  real gamma_tu = gam[3 * data_container->num_contacts + index * 3 + 1];
  real gamma_tv = gam[3 * data_container->num_contacts + index * 3 + 2];

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
  // gam[index + number_of_contacts * 0] = gamma_n;
  gam[3 * data_container->num_contacts + index * 3 + 0] = gamma_s;
  gam[3 * data_container->num_contacts + index * 3 + 1] = gamma_tu;
  gam[3 * data_container->num_contacts + index * 3 + 2] = gamma_tv;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::host_Project_single(int index, int2* ids, real3* friction, real* cohesion, real* gamma) {
  // always project normal
  switch (data_container->settings.solver.local_solver_mode) {
    case NORMAL: {
      func_Project_normal(index, ids, cohesion, gamma);
    } break;

    case SLIDING: {
      func_Project_sliding(index, ids, friction, cohesion, gamma);
    } break;

    case SPINNING: {
      func_Project_sliding(index, ids, friction, cohesion, gamma);
      func_Project_spinning(index, ids, friction, gamma);
    } break;
  }
}

void ChConstraintRigidRigid::Project(real* gamma) {
  const thrust::host_vector<int2> & bids = data_container->host_data.bids_rigid_rigid;
  const thrust::host_vector<real3> & friction = data_container->host_data.fric_rigid_rigid;
  const thrust::host_vector<real> & cohesion = data_container->host_data.coh_rigid_rigid;

  switch (data_container->settings.solver.local_solver_mode) {
    case NORMAL: {
#pragma omp parallel for
      for (int index = 0; index < data_container->num_contacts; index++) {
        func_Project_normal(index, bids.data(), cohesion.data(), gamma);
      }
    } break;

    case SLIDING: {
#pragma omp parallel for
      for (int index = 0; index < data_container->num_contacts; index++) {
        func_Project_sliding(index, bids.data(), friction.data(), cohesion.data(), gamma);
      }
    } break;

    case SPINNING: {
#pragma omp parallel for
      for (int index = 0; index < data_container->num_contacts; index++) {
        func_Project_sliding(index, bids.data(), friction.data(), cohesion.data(), gamma);
        func_Project_spinning(index, bids.data(), friction.data(), gamma);
      }
    } break;
  }
}
void ChConstraintRigidRigid::Project_Single(int index, real* gamma) {

  thrust::host_vector<int2>& bids = data_container->host_data.bids_rigid_rigid;
  thrust::host_vector<real3>& friction = data_container->host_data.fric_rigid_rigid;
  thrust::host_vector<real>& cohesion = data_container->host_data.coh_rigid_rigid;

  host_Project_single(index, bids.data(), friction.data(), cohesion.data(), gamma);
}

void chrono::Compute_Jacobian(const real4& quat, const real3& U, const real3& V, const real3& W, const real3& point, real3& T1, real3& T2, real3& T3) {
  real4 quaternion_conjugate = ~quat;
  real3 sbar = quatRotate(point, quaternion_conjugate);

  T1 = cross(quatRotate(U, quaternion_conjugate), sbar);
  T2 = cross(quatRotate(V, quaternion_conjugate), sbar);
  T3 = cross(quatRotate(W, quaternion_conjugate), sbar);
}
void chrono::Compute_Jacobian_Rolling(const real4& quat, const real3& U, const real3& V, const real3& W, real3& T1, real3& T2, real3& T3) {
  real4 quaternion_conjugate = ~quat;

  T1 = quatRotate(U, quaternion_conjugate);
  T2 = quatRotate(V, quaternion_conjugate);
  T3 = quatRotate(W, quaternion_conjugate);
}

void ChConstraintRigidRigid::Build_b()
{
  if (data_container->num_contacts <= 0) {
    return;
  }

#pragma omp parallel for
  for (int index = 0; index < data_container->num_contacts; index++) {
    real bi = 0;
    real depth = data_container->host_data.dpth_rigid_rigid[index];

    if (data_container->settings.solver.alpha > 0) {
      bi = inv_hpa * depth;
    } else if (data_container->settings.solver.contact_recovery_speed < 0) {
      bi = inv_h * depth;
    } else {
      bi = std::max(inv_h * depth, -data_container->settings.solver.contact_recovery_speed);
    }


    data_container->host_data.b[index * 1 + 0] = bi;
  }
}

void ChConstraintRigidRigid::Build_s()
{
  if (data_container->num_contacts <= 0) {
    return;
  }

  if( data_container->settings.solver.solver_mode == NORMAL){
    return;
  }

  int2* ids = data_container->host_data.bids_rigid_rigid.data();
  const CompressedMatrix<real>& D_t_T = data_container->host_data.D_t_T;
  DynamicVector<real>& v = data_container->host_data.v;

  const DynamicVector<real> v_new = data_container->host_data.M_invk + data_container->host_data.M_invD * data_container->host_data.gamma;

#pragma omp parallel for
  for (int index = 0; index < data_container->num_contacts; index++) {
    real fric = data_container->host_data.fric_rigid_rigid[index].x;
    int index_mult = _index_;
    int2 body_id = ids[index];



//    blaze::SparseSubmatrix< CompressedMatrix<real> > D_v_a = submatrix( D_T, index_mult + 1, body_id.x* 6 + 0, 1, 6 );
//    blaze::SparseSubmatrix< CompressedMatrix<real> > D_v_b = submatrix( D_T, index_mult + 1, body_id.y* 6 + 0, 1, 6 );
//
//    blaze::SparseSubmatrix< CompressedMatrix<real> > D_w_a = submatrix( D_T, index_mult + 2, body_id.x* 6 + 0, 1, 6 );
//    blaze::SparseSubmatrix< CompressedMatrix<real> > D_w_b = submatrix( D_T, index_mult + 2, body_id.y* 6 + 0, 1, 6 );
//
//
//    blaze::DenseSubvector<DynamicVector<real> > v_a = blaze::subvector(v_new, body_id.x* 6 + 0, 6);
//    blaze::DenseSubvector<DynamicVector<real> > v_b = blaze::subvector(v_new, body_id.y* 6 + 0, 6);
//    real s_v, s_w;

    real s_v =
        D_t_T(index * 2 + 0, body_id.x* 6 + 0)* +v_new[body_id.x* 6 + 0]+
        D_t_T(index * 2 + 0, body_id.x* 6 + 1)* +v_new[body_id.x* 6 + 1]+
        D_t_T(index * 2 + 0, body_id.x* 6 + 2)* +v_new[body_id.x* 6 + 2]+
        D_t_T(index * 2 + 0, body_id.x* 6 + 3)* +v_new[body_id.x* 6 + 3]+
        D_t_T(index * 2 + 0, body_id.x* 6 + 4)* +v_new[body_id.x* 6 + 4]+
        D_t_T(index * 2 + 0, body_id.x* 6 + 5)* +v_new[body_id.x* 6 + 5]+

        D_t_T(index * 2 + 0, body_id.y* 6 + 0)* +v_new[body_id.y* 6 + 0]+
        D_t_T(index * 2 + 0, body_id.y* 6 + 1)* +v_new[body_id.y* 6 + 1]+
        D_t_T(index * 2 + 0, body_id.y* 6 + 2)* +v_new[body_id.y* 6 + 2]+
        D_t_T(index * 2 + 0, body_id.y* 6 + 3)* +v_new[body_id.y* 6 + 3]+
        D_t_T(index * 2 + 0, body_id.y* 6 + 4)* +v_new[body_id.y* 6 + 4]+
        D_t_T(index * 2 + 0, body_id.y* 6 + 5)* +v_new[body_id.y* 6 + 5];

    real s_w =
        D_t_T(index * 2 + 1, body_id.x* 6 + 0)* +v_new[body_id.x* 6 + 0]+
        D_t_T(index * 2 + 1, body_id.x* 6 + 1)* +v_new[body_id.x* 6 + 1]+
        D_t_T(index * 2 + 1, body_id.x* 6 + 2)* +v_new[body_id.x* 6 + 2]+
        D_t_T(index * 2 + 1, body_id.x* 6 + 3)* +v_new[body_id.x* 6 + 3]+
        D_t_T(index * 2 + 1, body_id.x* 6 + 4)* +v_new[body_id.x* 6 + 4]+
        D_t_T(index * 2 + 1, body_id.x* 6 + 5)* +v_new[body_id.x* 6 + 5]+

        D_t_T(index * 2 + 1, body_id.y* 6 + 0)* +v_new[body_id.y* 6 + 0]+
        D_t_T(index * 2 + 1, body_id.y* 6 + 1)* +v_new[body_id.y* 6 + 1]+
        D_t_T(index * 2 + 1, body_id.y* 6 + 2)* +v_new[body_id.y* 6 + 2]+
        D_t_T(index * 2 + 1, body_id.y* 6 + 3)* +v_new[body_id.y* 6 + 3]+
        D_t_T(index * 2 + 1, body_id.y* 6 + 4)* +v_new[body_id.y* 6 + 4]+
        D_t_T(index * 2 + 1, body_id.y* 6 + 5)* +v_new[body_id.y* 6 + 5];


    data_container->host_data.s[index * 1 + 0] = sqrt(s_v* s_v + s_w* s_w) * fric;

    //std::cout<<s_v<<" "<<s_w<<" "<<data_container->host_data.s[_index_ + 0]<<std::endl;
  }
}

void ChConstraintRigidRigid::Build_E()
{
  if (data_container->num_contacts <= 0) {
    return;
  }
  SOLVERMODE solver_mode = data_container->settings.solver.solver_mode;
  DynamicVector<real>& E = data_container->host_data.E;
  uint num_contacts = data_container->num_contacts;

#pragma omp parallel for
  for (int index = 0; index < data_container->num_contacts; index++) {
    int2 body = data_container->host_data.bids_rigid_rigid[index];

    real4 cA = data_container->host_data.compliance_data[body.x];
    real4 cB = data_container->host_data.compliance_data[body.y];
    real4 compliance;

    compliance.x = (cA.x == 0 || cB.x == 0) ? 0 : (cA.x + cB.x) * .5;
    compliance.y = (cA.y == 0 || cB.y == 0) ? 0 : (cA.y + cB.y) * .5;
    compliance.z = (cA.z == 0 || cB.z == 0) ? 0 : (cA.z + cB.z) * .5;
    compliance.w = (cA.w == 0 || cB.w == 0) ? 0 : (cA.w + cB.w) * .5;

    E[index * 1 + 0] = compliance.x;    // normal
    if (solver_mode == SLIDING) {
      E[num_contacts + index * 2 + 0] = compliance.y;    // sliding
      E[num_contacts + index * 2 + 1] = compliance.y;    // sliding
    } else if (solver_mode == SPINNING) {
      E[3 * num_contacts + index * 3 + 0] = compliance.w;    // sliding
      E[3 * num_contacts + index * 3 + 1] = compliance.z;    // sliding
      E[3 * num_contacts + index * 3 + 2] = compliance.z;    // sliding
    }
  }
}

void ChConstraintRigidRigid::Build_D()
{
  real3* norm = data_container->host_data.norm_rigid_rigid.data();
  real3* ptA = data_container->host_data.cpta_rigid_rigid.data();
  real3* ptB = data_container->host_data.cptb_rigid_rigid.data();
  int2* ids = data_container->host_data.bids_rigid_rigid.data();
  real4* rot = contact_rotation.data();

  CompressedMatrix<real>& D_n_T = data_container->host_data.D_n_T;
  CompressedMatrix<real>& D_t_T = data_container->host_data.D_t_T;
  CompressedMatrix<real>& D_s_T = data_container->host_data.D_s_T;

  SOLVERMODE solver_mode = data_container->settings.solver.solver_mode;

#pragma omp parallel for
  for (int index = 0; index < data_container->num_contacts; index++) {
    real3 U = norm[index], V, W;
    real3 T3, T4, T5, T6, T7, T8;
    real3 TA, TB, TC;
    real3 TD, TE, TF;

    Orthogonalize(U, V, W);

    int2 body_id = ids[index];

    int row = index;

    if (solver_mode == SLIDING || solver_mode == SPINNING) {
      Compute_Jacobian(rot[index], U, V, W, ptA[index], T3, T4, T5);
      Compute_Jacobian(rot[index + data_container->num_contacts], U, V, W, ptB[index], T6, T7, T8);
    }
    if (contact_active_pairs[index].x) {
      D_n_T(row * 1 + 0, body_id.x* 6 + 0) = -U.x;
      D_n_T(row * 1 + 0, body_id.x* 6 + 1) = -U.y;
      D_n_T(row * 1 + 0, body_id.x* 6 + 2) = -U.z;

      D_n_T(row * 1 + 0, body_id.x* 6 + 3) = T3.x;
      D_n_T(row * 1 + 0, body_id.x* 6 + 4) = T3.y;
      D_n_T(row * 1 + 0, body_id.x* 6 + 5) = T3.z;
    }
    if (contact_active_pairs[index].y) {
      D_n_T(row * 1 + 0, body_id.y* 6 + 0) = U.x;
      D_n_T(row * 1 + 0, body_id.y* 6 + 1) = U.y;
      D_n_T(row * 1 + 0, body_id.y* 6 + 2) = U.z;

      D_n_T(row * 1 + 0, body_id.y* 6 + 3) = -T6.x;
      D_n_T(row * 1 + 0, body_id.y* 6 + 4) = -T6.y;
      D_n_T(row * 1 + 0, body_id.y* 6 + 5) = -T6.z;
    }

    if (solver_mode == SLIDING || solver_mode == SPINNING) {
      if (contact_active_pairs[index].x) {
        D_t_T(row * 2 + 0, body_id.x* 6 + 0) = -V.x;
        D_t_T(row * 2 + 0, body_id.x* 6 + 1) = -V.y;
        D_t_T(row * 2 + 0, body_id.x* 6 + 2) = -V.z;

        D_t_T(row * 2 + 0, body_id.x* 6 + 3) = T4.x;
        D_t_T(row * 2 + 0, body_id.x* 6 + 4) = T4.y;
        D_t_T(row * 2 + 0, body_id.x* 6 + 5) = T4.z;

        D_t_T(row * 2 + 1, body_id.x* 6 + 0) = -W.x;
        D_t_T(row * 2 + 1, body_id.x* 6 + 1) = -W.y;
        D_t_T(row * 2 + 1, body_id.x* 6 + 2) = -W.z;

        D_t_T(row * 2 + 1, body_id.x* 6 + 3) = T5.x;
        D_t_T(row * 2 + 1, body_id.x* 6 + 4) = T5.y;
        D_t_T(row * 2 + 1, body_id.x* 6 + 5) = T5.z;
      }
      if (contact_active_pairs[index].y) {
        D_t_T(row * 2 + 0, body_id.y* 6 + 0) = V.x;
        D_t_T(row * 2 + 0, body_id.y* 6 + 1) = V.y;
        D_t_T(row * 2 + 0, body_id.y* 6 + 2) = V.z;

        D_t_T(row * 2 + 0, body_id.y* 6 + 3) = -T7.x;
        D_t_T(row * 2 + 0, body_id.y* 6 + 4) = -T7.y;
        D_t_T(row * 2 + 0, body_id.y* 6 + 5) = -T7.z;

        D_t_T(row * 2 + 1, body_id.y* 6 + 0) = W.x;
        D_t_T(row * 2 + 1, body_id.y* 6 + 1) = W.y;
        D_t_T(row * 2 + 1, body_id.y* 6 + 2) = W.z;

        D_t_T(row * 2 + 1, body_id.y* 6 + 3) = -T8.x;
        D_t_T(row * 2 + 1, body_id.y* 6 + 4) = -T8.y;
        D_t_T(row * 2 + 1, body_id.y* 6 + 5) = -T8.z;
      }
    }
    if (solver_mode == SPINNING) {
      Compute_Jacobian_Rolling(rot[index], U, V, W, TA, TB, TC);
      Compute_Jacobian_Rolling(rot[index + data_container->num_contacts], U, V, W, TD, TE, TF);
      if (contact_active_pairs[index].x) {
        D_s_T(row * 3 + 0, body_id.x* 6 + 3) = -TA.x;
        D_s_T(row * 3 + 0, body_id.x* 6 + 4) = -TA.y;
        D_s_T(row * 3 + 0, body_id.x* 6 + 5) = -TA.z;

        D_s_T(row * 3 + 1, body_id.x* 6 + 3) = -TB.x;
        D_s_T(row * 3 + 1, body_id.x* 6 + 4) = -TB.y;
        D_s_T(row * 3 + 1, body_id.x* 6 + 5) = -TB.z;

        D_s_T(row * 3 + 2, body_id.x* 6 + 3) = -TC.x;
        D_s_T(row * 3 + 2, body_id.x* 6 + 4) = -TC.y;
        D_s_T(row * 3 + 2, body_id.x* 6 + 5) = -TC.z;
      }
      if (contact_active_pairs[index].y) {
        D_s_T(row * 3 + 0, body_id.y* 6 + 3) = TD.x;
        D_s_T(row * 3 + 0, body_id.y* 6 + 4) = TD.y;
        D_s_T(row * 3 + 0, body_id.y* 6 + 5) = TD.z;

        D_s_T(row * 3 + 1, body_id.y* 6 + 3) = TE.x;
        D_s_T(row * 3 + 1, body_id.y* 6 + 4) = TE.y;
        D_s_T(row * 3 + 1, body_id.y* 6 + 5) = TE.z;

        D_s_T(row * 3 + 2, body_id.y* 6 + 3) = TF.x;
        D_s_T(row * 3 + 2, body_id.y* 6 + 4) = TF.y;
        D_s_T(row * 3 + 2, body_id.y* 6 + 5) = TF.z;
      }
    }
  }
}

void ChConstraintRigidRigid::GenerateSparsity()
{
  SOLVERMODE solver_mode = data_container->settings.solver.solver_mode;

  CompressedMatrix<real>& D_n_T = data_container->host_data.D_n_T;
  CompressedMatrix<real>& D_t_T = data_container->host_data.D_t_T;
  CompressedMatrix<real>& D_s_T = data_container->host_data.D_s_T;

  const int2* ids = data_container->host_data.bids_rigid_rigid.data();

  for (int index = 0; index < data_container->num_contacts; index++) {
    int2 body_id = ids[index];
    int row = index;

    if (contact_active_pairs[index].x) {
      D_n_T.append(row * 1 + 0, body_id.x * 6 + 0, 1);
      D_n_T.append(row * 1 + 0, body_id.x * 6 + 1, 1);
      D_n_T.append(row * 1 + 0, body_id.x * 6 + 2, 1);

      D_n_T.append(row * 1 + 0, body_id.x * 6 + 3, 1);
      D_n_T.append(row * 1 + 0, body_id.x * 6 + 4, 1);
      D_n_T.append(row * 1 + 0, body_id.x * 6 + 5, 1);
    }
    if (contact_active_pairs[index].y) {
      D_n_T.append(row * 1 + 0, body_id.y * 6 + 0, 1);
      D_n_T.append(row * 1 + 0, body_id.y * 6 + 1, 1);
      D_n_T.append(row * 1 + 0, body_id.y * 6 + 2, 1);

      D_n_T.append(row * 1 + 0, body_id.y * 6 + 3, 1);
      D_n_T.append(row * 1 + 0, body_id.y * 6 + 4, 1);
      D_n_T.append(row * 1 + 0, body_id.y * 6 + 5, 1);
    }
    D_n_T.finalize(row * 1 + 0);

    if (solver_mode == SLIDING || solver_mode == SPINNING) {
      if (contact_active_pairs[index].x) {
        D_t_T.append(row * 2 + 0, body_id.x * 6 + 0, 1);
        D_t_T.append(row * 2 + 0, body_id.x * 6 + 1, 1);
        D_t_T.append(row * 2 + 0, body_id.x * 6 + 2, 1);

        D_t_T.append(row * 2 + 0, body_id.x * 6 + 3, 1);
        D_t_T.append(row * 2 + 0, body_id.x * 6 + 4, 1);
        D_t_T.append(row * 2 + 0, body_id.x * 6 + 5, 1);
      }
      if (contact_active_pairs[index].y) {
        D_t_T.append(row * 2 + 0, body_id.y * 6 + 0, 1);
        D_t_T.append(row * 2 + 0, body_id.y * 6 + 1, 1);
        D_t_T.append(row * 2 + 0, body_id.y * 6 + 2, 1);

        D_t_T.append(row * 2 + 0, body_id.y * 6 + 3, 1);
        D_t_T.append(row * 2 + 0, body_id.y * 6 + 4, 1);
        D_t_T.append(row * 2 + 0, body_id.y * 6 + 5, 1);
      }
      D_t_T.finalize(row * 2 + 0);

      if (contact_active_pairs[index].x) {
        D_t_T.append(row * 2 + 1, body_id.x * 6 + 0, 1);
        D_t_T.append(row * 2 + 1, body_id.x * 6 + 1, 1);
        D_t_T.append(row * 2 + 1, body_id.x * 6 + 2, 1);

        D_t_T.append(row * 2 + 1, body_id.x * 6 + 3, 1);
        D_t_T.append(row * 2 + 1, body_id.x * 6 + 4, 1);
        D_t_T.append(row * 2 + 1, body_id.x * 6 + 5, 1);
      }
      if (contact_active_pairs[index].y) {
        D_t_T.append(row * 2 + 1, body_id.y * 6 + 0, 1);
        D_t_T.append(row * 2 + 1, body_id.y * 6 + 1, 1);
        D_t_T.append(row * 2 + 1, body_id.y * 6 + 2, 1);

        D_t_T.append(row * 2 + 1, body_id.y * 6 + 3, 1);
        D_t_T.append(row * 2 + 1, body_id.y * 6 + 4, 1);
        D_t_T.append(row * 2 + 1, body_id.y * 6 + 5, 1);
      }
      D_t_T.finalize(row * 2 + 1);
    }

    if (solver_mode == SPINNING) {
      if (contact_active_pairs[index].x) {
        D_s_T.append(row * 3 + 0, body_id.x * 6 + 3, 1);
        D_s_T.append(row * 3 + 0, body_id.x * 6 + 4, 1);
        D_s_T.append(row * 3 + 0, body_id.x * 6 + 5, 1);
      }
      if (contact_active_pairs[index].y) {
        D_s_T.append(row * 3 + 0, body_id.y * 6 + 3, 1);
        D_s_T.append(row * 3 + 0, body_id.y * 6 + 4, 1);
        D_s_T.append(row * 3 + 0, body_id.y * 6 + 5, 1);
      }
      D_s_T.finalize(row * 3 + 0);

      if (contact_active_pairs[index].x) {
        D_s_T.append(row * 3 + 1, body_id.x * 6 + 3, 1);
        D_s_T.append(row * 3 + 1, body_id.x * 6 + 4, 1);
        D_s_T.append(row * 3 + 1, body_id.x * 6 + 5, 1);
      }
      if (contact_active_pairs[index].y) {
        D_s_T.append(row * 3 + 1, body_id.y * 6 + 3, 1);
        D_s_T.append(row * 3 + 1, body_id.y * 6 + 4, 1);
        D_s_T.append(row * 3 + 1, body_id.y * 6 + 5, 1);
      }
      D_s_T.finalize(row * 3 + 1);
      
      if (contact_active_pairs[index].x) {
        D_s_T.append(row * 3 + 2, body_id.x * 6 + 3, 1);
        D_s_T.append(row * 3 + 2, body_id.x * 6 + 4, 1);
        D_s_T.append(row * 3 + 2, body_id.x * 6 + 5, 1);
      }
      if (contact_active_pairs[index].y) {
        D_s_T.append(row * 3 + 2, body_id.y * 6 + 3, 1);
        D_s_T.append(row * 3 + 2, body_id.y * 6 + 4, 1);
        D_s_T.append(row * 3 + 2, body_id.y * 6 + 5, 1);
      }
      D_s_T.finalize(row * 3 + 2);
    }
  }
}
