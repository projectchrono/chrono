#include <algorithm>

#include "chrono_parallel/ChConfigParallel.h"
#include "chrono_parallel/constraints/ChConstraintRigidRigid.h"

using namespace chrono;
#define _index_ index* offset

void chrono::Orthogonalize(real3& Vx, real3& Vy, real3& Vz) {
  real3 mVsingular = R3(0, 1, 0);
  Vz = cross(Vx, mVsingular);
  real mzlen = Vz.length();

  if (mzlen < real(0.0001)) {    // was near singularity? change singularity
                                 // reference custom_vector!

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

void ChConstraintRigidRigid::func_Project(int& index, int2* ids, real3* fric, real* cohesion, real* gam) {
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
void ChConstraintRigidRigid::func_Project_rolling(int& index, int2* ids, real3* fric, real* gam) {
  // real3 gamma_roll = R3(0);
  real rollingfriction = fric[index].y;
  real spinningfriction = fric[index].z;

  //	if(rollingfriction||spinningfriction){
  //		gam[index + number_of_contacts * 1] = 0;
  //		gam[index + number_of_contacts * 2] = 0;
  //	}

  real gamma_n = fabs(gam[_index_ + 0]);
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
  // gam[index + number_of_contacts * 0] = gamma_n;
  gam[_index_ + 3] = gamma_s;
  gam[_index_ + 4] = gamma_tu;
  gam[_index_ + 5] = gamma_tv;
}

void ChConstraintRigidRigid::host_Project_single(int index, int2* ids, real3* friction, real* cohesion, real* gamma) {
  // always project normal
  if (solve_sliding) {
    func_Project(index, ids, friction, cohesion, gamma);
  } else {
    real gamma_x = gamma[_index_ + 0];
    int2 body_id = ids[index];
    real coh = cohesion[index];

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

void ChConstraintRigidRigid::host_Project(int2* ids, real3* friction, real* cohesion, real* gamma) {
  // always project normal
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
      real coh = cohesion[index];

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
  host_Project(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.fric_rigid_rigid.data(), data_container->host_data.coh_rigid_rigid.data(), gamma);
}
void ChConstraintRigidRigid::Project_NoPar(real* gamma) {
  host_Project(data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.fric_rigid_rigid.data(), data_container->host_data.coh_rigid_rigid.data(), gamma);
}
void ChConstraintRigidRigid::Project_Single(int index, real* gamma) {
  host_Project_single(index, data_container->host_data.bids_rigid_rigid.data(), data_container->host_data.fric_rigid_rigid.data(), data_container->host_data.coh_rigid_rigid.data(), gamma);
}

void chrono::Compute_Jacobian(const real4& quat, const real3& U, const real3& V, const real3& W, const real3& point, real3& T1, real3& T2, real3& T3) {
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
void chrono::Compute_Jacobian_Rolling(const real4& quat, const real3& U, const real3& V, const real3& W, real3& T1, real3& T2, real3& T3) {
  real4 quaternion_conjugate = ~quat;

  T1 = quatRotate(U, quaternion_conjugate);
  T2 = quatRotate(V, quaternion_conjugate);
  T3 = quatRotate(W, quaternion_conjugate);
}

void ChConstraintRigidRigid::host_ComputeS(int2* ids, real3* mu, bool2* active, real3* norm, real3* vel, real3* omega, real3* ptA, real3* ptB, real4* rot, const real* rhs, real* b) {
#pragma omp parallel for
  for (int index = 0; index < num_contacts; index++) {
    int2 bid = ids[index];
    bool2 isactive = active[index];

    real3 temp = R3(0);

    real3 U = norm[index], V, W;
    Orthogonalize(U, V, W);    // read 3 real

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
    // cout<<"here: "<<s<<endl;
  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::ComputeS(const custom_vector<real>& rhs, custom_vector<real3>& vel_data, custom_vector<real3>& omg_data, custom_vector<real>& b) {
  //   if (solve_sliding) {
  //      host_ComputeS(data_container->host_data.bids_rigid_rigid.data(),
  //      data_container->host_data.fric_data.data(),
  //      contact_active_pairs.data(),
  //                    data_container->host_data.norm_rigid_rigid.data(),
  //                    vel_data.data(), omg_data.data(),
  //                    data_container->host_data.cpta_rigid_rigid.data(),
  //                    data_container->host_data.cptb_rigid_rigid.data(),
  //                    contact_rotation.data(), rhs.data(), b.data());
  //   }
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::Build_b(SOLVERMODE solver_mode) {
  if (num_contacts <= 0) {
    return;
  }
#pragma omp parallel for
  for (int index = 0; index < num_contacts; index++) {
    real bi = 0;
    real depth = data_container->host_data.dpth_rigid_rigid[index];
    if (data_container->settings.solver.alpha > 0) {
      bi = inv_hpa * depth;
    } else {
      if (data_container->settings.solver.contact_recovery_speed < 0) {
        bi = real(1.0) / step_size * depth;
      } else {
        bi = std::max(real(1.0) / step_size * depth, -data_container->settings.solver.contact_recovery_speed);
      }
    }

    data_container->host_data.b[_index_ + 0] = bi;
    //
    //      if (solver_mode == SLIDING) {
    //         data_container->host_data.b[_index_ + 1] = 0;
    //         data_container->host_data.b[_index_ + 2] = 0;
    //      } else if (solver_mode == SPINNING) {
    //         data_container->host_data.b[_index_ + 3] = 0;
    //         data_container->host_data.b[_index_ + 4] = 0;
    //         data_container->host_data.b[_index_ + 5] = 0;
    //      }
  }
}
void ChConstraintRigidRigid::Build_E(SOLVERMODE solver_mode) {
  if (num_contacts <= 0) {
    return;
  }
  DynamicVector<real>& E = data_container->host_data.E;
#pragma omp parallel for
  for (int index = 0; index < num_contacts; index++) {
    int2 body = data_container->host_data.bids_rigid_rigid[index];

    real4 cA = data_container->host_data.compliance_data[body.x];
    real4 cB = data_container->host_data.compliance_data[body.y];
    real4 compliance;

    compliance.x = (cA.x == 0 || cB.x == 0) ? 0 : (cA.x + cB.x) * .5;
    compliance.y = (cA.y == 0 || cB.y == 0) ? 0 : (cA.y + cB.y) * .5;
    compliance.z = (cA.z == 0 || cB.z == 0) ? 0 : (cA.z + cB.z) * .5;
    compliance.w = (cA.w == 0 || cB.w == 0) ? 0 : (cA.w + cB.w) * .5;

    E[_index_] = compliance.x;    // normal
    if (solver_mode == SLIDING) {
      E[_index_ + 1] = compliance.y;    // sliding
      E[_index_ + 2] = compliance.y;    // sliding
    } else if (solver_mode == SPINNING) {
      E[_index_ + 3] = compliance.w;    // sliding
      E[_index_ + 4] = compliance.z;    // sliding
      E[_index_ + 5] = compliance.z;    // sliding
    }
  }
}
void ChConstraintRigidRigid::Build_D(SOLVERMODE solver_mode) {
  real3* norm = data_container->host_data.norm_rigid_rigid.data();
  real3* ptA = data_container->host_data.cpta_rigid_rigid.data();
  real3* ptB = data_container->host_data.cptb_rigid_rigid.data();
  int2* ids = data_container->host_data.bids_rigid_rigid.data();
  real4* rot = contact_rotation.data();
  CompressedMatrix<real>& D_T = data_container->host_data.D_T;

#pragma omp parallel for
  for (int index = 0; index < num_contacts; index++) {
    real3 U = norm[index], V, W;
    real3 T3, T4, T5, T6, T7, T8;
    real3 TA, TB, TC;
    real3 TD, TE, TF;

    Orthogonalize(U, V, W);

    int2 body_id = ids[index];

    int index_mult = _index_;

    if (solver_mode == SLIDING || solver_mode == SPINNING) {
      Compute_Jacobian(rot[index], U, V, W, ptA[index], T3, T4, T5);
      Compute_Jacobian(rot[index + num_contacts], U, V, W, ptB[index], T6, T7, T8);
    }
    if (contact_active_pairs[index].x) {
      D_T(index_mult + 0, body_id.x* 6 + 0) = -U.x;
      D_T(index_mult + 0, body_id.x* 6 + 1) = -U.y;
      D_T(index_mult + 0, body_id.x* 6 + 2) = -U.z;

      if (solver_mode == SLIDING || solver_mode == SPINNING) {
        D_T(index_mult + 0, body_id.x* 6 + 3) = T3.x;
        D_T(index_mult + 0, body_id.x* 6 + 4) = T3.y;
        D_T(index_mult + 0, body_id.x* 6 + 5) = T3.z;
      }
    }
    if (contact_active_pairs[index].y) {
      D_T(index_mult + 0, body_id.y* 6 + 0) = U.x;
      D_T(index_mult + 0, body_id.y* 6 + 1) = U.y;
      D_T(index_mult + 0, body_id.y* 6 + 2) = U.z;
      if (solver_mode == SLIDING || solver_mode == SPINNING) {
        D_T(index_mult + 0, body_id.y* 6 + 3) = -T6.x;
        D_T(index_mult + 0, body_id.y* 6 + 4) = -T6.y;
        D_T(index_mult + 0, body_id.y* 6 + 5) = -T6.z;
      }
    }
    if (contact_active_pairs[index].x) {
      D_T(index_mult + 1, body_id.x* 6 + 0) = -V.x;
      D_T(index_mult + 1, body_id.x* 6 + 1) = -V.y;
      D_T(index_mult + 1, body_id.x* 6 + 2) = -V.z;

      if (solver_mode == SLIDING || solver_mode == SPINNING) {
        D_T(index_mult + 1, body_id.x* 6 + 3) = T4.x;
        D_T(index_mult + 1, body_id.x* 6 + 4) = T4.y;
        D_T(index_mult + 1, body_id.x* 6 + 5) = T4.z;
      }
    }
    if (contact_active_pairs[index].y) {
      D_T(index_mult + 1, body_id.y* 6 + 0) = V.x;
      D_T(index_mult + 1, body_id.y* 6 + 1) = V.y;
      D_T(index_mult + 1, body_id.y* 6 + 2) = V.z;

      if (solver_mode == SLIDING || solver_mode == SPINNING) {
        D_T(index_mult + 1, body_id.y* 6 + 3) = -T7.x;
        D_T(index_mult + 1, body_id.y* 6 + 4) = -T7.y;
        D_T(index_mult + 1, body_id.y* 6 + 5) = -T7.z;
      }
    }
    if (contact_active_pairs[index].x) {
      D_T(index_mult + 2, body_id.x* 6 + 0) = -W.x;
      D_T(index_mult + 2, body_id.x* 6 + 1) = -W.y;
      D_T(index_mult + 2, body_id.x* 6 + 2) = -W.z;

      if (solver_mode == SLIDING || solver_mode == SPINNING) {
        D_T(index_mult + 2, body_id.x* 6 + 3) = T5.x;
        D_T(index_mult + 2, body_id.x* 6 + 4) = T5.y;
        D_T(index_mult + 2, body_id.x* 6 + 5) = T5.z;
      }
    }
    if (contact_active_pairs[index].y) {
      D_T(index_mult + 2, body_id.y* 6 + 0) = W.x;
      D_T(index_mult + 2, body_id.y* 6 + 1) = W.y;
      D_T(index_mult + 2, body_id.y* 6 + 2) = W.z;

      if (solver_mode == SLIDING || solver_mode == SPINNING) {
        D_T(index_mult + 2, body_id.y* 6 + 3) = -T8.x;
        D_T(index_mult + 2, body_id.y* 6 + 4) = -T8.y;
        D_T(index_mult + 2, body_id.y* 6 + 5) = -T8.z;
      }
    }
    if (solver_mode == SPINNING) {
      Compute_Jacobian_Rolling(rot[index], U, V, W, TA, TB, TC);
      Compute_Jacobian_Rolling(rot[index + num_contacts], U, V, W, TD, TE, TF);
      if (contact_active_pairs[index].x) {
        D_T(index_mult + 3, body_id.x* 6 + 3) = -TA.x;
        D_T(index_mult + 3, body_id.x* 6 + 4) = -TA.y;
        D_T(index_mult + 3, body_id.x* 6 + 5) = -TA.z;
      }
      if (contact_active_pairs[index].y) {
        D_T(index_mult + 3, body_id.y* 6 + 3) = TD.x;
        D_T(index_mult + 3, body_id.y* 6 + 4) = TD.y;
        D_T(index_mult + 3, body_id.y* 6 + 5) = TD.z;
      }
      if (contact_active_pairs[index].x) {
        D_T(index_mult + 4, body_id.x* 6 + 3) = -TB.x;
        D_T(index_mult + 4, body_id.x* 6 + 4) = -TB.y;
        D_T(index_mult + 4, body_id.x* 6 + 5) = -TB.z;
      }
      if (contact_active_pairs[index].y) {
        D_T(index_mult + 4, body_id.y* 6 + 3) = TE.x;
        D_T(index_mult + 4, body_id.y* 6 + 4) = TE.y;
        D_T(index_mult + 4, body_id.y* 6 + 5) = TE.z;
      }
      if (contact_active_pairs[index].x) {
        D_T(index_mult + 5, body_id.x* 6 + 3) = -TC.x;
        D_T(index_mult + 5, body_id.x* 6 + 4) = -TC.y;
        D_T(index_mult + 5, body_id.x* 6 + 5) = -TC.z;
      }
      if (contact_active_pairs[index].y) {
        D_T(index_mult + 5, body_id.y* 6 + 3) = TF.x;
        D_T(index_mult + 5, body_id.y* 6 + 4) = TF.y;
        D_T(index_mult + 5, body_id.y* 6 + 5) = TF.z;
      }
    }
  }
}

void ChConstraintRigidRigid::GenerateSparsity(SOLVERMODE solver_mode) {
  CompressedMatrix<real>& D_T = data_container->host_data.D_T;
  int2* ids = data_container->host_data.bids_rigid_rigid.data();

  for (int index = 0; index < num_contacts; index++) {
    int2 body_id = ids[index];
    int index_mult = _index_;
    if (contact_active_pairs[index].x) {
      D_T.append(index_mult + 0, body_id.x * 6 + 0, 0);
      D_T.append(index_mult + 0, body_id.x * 6 + 1, 0);
      D_T.append(index_mult + 0, body_id.x * 6 + 2, 0);
      if (solver_mode == SLIDING || solver_mode == SPINNING) {
        D_T.append(index_mult + 0, body_id.x * 6 + 3, 0);
        D_T.append(index_mult + 0, body_id.x * 6 + 4, 0);
        D_T.append(index_mult + 0, body_id.x * 6 + 5, 0);
      }
    }
    if (contact_active_pairs[index].y) {
      D_T.append(index_mult + 0, body_id.y * 6 + 0, 0);
      D_T.append(index_mult + 0, body_id.y * 6 + 1, 0);
      D_T.append(index_mult + 0, body_id.y * 6 + 2, 0);
      if (solver_mode == SLIDING || solver_mode == SPINNING) {
        D_T.append(index_mult + 0, body_id.y * 6 + 3, 0);
        D_T.append(index_mult + 0, body_id.y * 6 + 4, 0);
        D_T.append(index_mult + 0, body_id.y * 6 + 5, 0);
      }
    }
    D_T.finalize(index_mult + 0);
    if (contact_active_pairs[index].x) {
      D_T.append(index_mult + 1, body_id.x * 6 + 0, 0);
      D_T.append(index_mult + 1, body_id.x * 6 + 1, 0);
      D_T.append(index_mult + 1, body_id.x * 6 + 2, 0);
      if (solver_mode == SLIDING || solver_mode == SPINNING) {
        D_T.append(index_mult + 1, body_id.x * 6 + 3, 0);
        D_T.append(index_mult + 1, body_id.x * 6 + 4, 0);
        D_T.append(index_mult + 1, body_id.x * 6 + 5, 0);
      }
    }
    if (contact_active_pairs[index].y) {
      D_T.append(index_mult + 1, body_id.y * 6 + 0, 0);
      D_T.append(index_mult + 1, body_id.y * 6 + 1, 0);
      D_T.append(index_mult + 1, body_id.y * 6 + 2, 0);
      if (solver_mode == SLIDING || solver_mode == SPINNING) {
        D_T.append(index_mult + 1, body_id.y * 6 + 3, 0);
        D_T.append(index_mult + 1, body_id.y * 6 + 4, 0);
        D_T.append(index_mult + 1, body_id.y * 6 + 5, 0);
      }
    }
    D_T.finalize(index_mult + 1);
    if (contact_active_pairs[index].x) {
      D_T.append(index_mult + 2, body_id.x * 6 + 0, 0);
      D_T.append(index_mult + 2, body_id.x * 6 + 1, 0);
      D_T.append(index_mult + 2, body_id.x * 6 + 2, 0);
      if (solver_mode == SLIDING || solver_mode == SPINNING) {
        D_T.append(index_mult + 2, body_id.x * 6 + 3, 0);
        D_T.append(index_mult + 2, body_id.x * 6 + 4, 0);
        D_T.append(index_mult + 2, body_id.x * 6 + 5, 0);
      }
    }
    if (contact_active_pairs[index].y) {
      D_T.append(index_mult + 2, body_id.y * 6 + 0, 0);
      D_T.append(index_mult + 2, body_id.y * 6 + 1, 0);
      D_T.append(index_mult + 2, body_id.y * 6 + 2, 0);
      if (solver_mode == SLIDING || solver_mode == SPINNING) {
        D_T.append(index_mult + 2, body_id.y * 6 + 3, 0);
        D_T.append(index_mult + 2, body_id.y * 6 + 4, 0);
        D_T.append(index_mult + 2, body_id.y * 6 + 5, 0);
      }
    }
    D_T.finalize(index_mult + 2);
    if (solver_mode == SPINNING) {
      if (contact_active_pairs[index].x) {
        D_T.append(index_mult + 3, body_id.x * 6 + 3, 0);
        D_T.append(index_mult + 3, body_id.x * 6 + 4, 0);
        D_T.append(index_mult + 3, body_id.x * 6 + 5, 0);
      }
      if (contact_active_pairs[index].y) {
        D_T.append(index_mult + 3, body_id.y * 6 + 3, 0);
        D_T.append(index_mult + 3, body_id.y * 6 + 4, 0);
        D_T.append(index_mult + 3, body_id.y * 6 + 5, 0);
      }
      D_T.finalize(index_mult + 3);
      if (contact_active_pairs[index].x) {
        D_T.append(index_mult + 4, body_id.x * 6 + 3, 0);
        D_T.append(index_mult + 4, body_id.x * 6 + 4, 0);
        D_T.append(index_mult + 4, body_id.x * 6 + 5, 0);
      }
      if (contact_active_pairs[index].y) {
        D_T.append(index_mult + 4, body_id.y * 6 + 3, 0);
        D_T.append(index_mult + 4, body_id.y * 6 + 4, 0);
        D_T.append(index_mult + 4, body_id.y * 6 + 5, 0);
      }
      D_T.finalize(index_mult + 4);
      if (contact_active_pairs[index].x) {
        D_T.append(index_mult + 5, body_id.x * 6 + 3, 0);
        D_T.append(index_mult + 5, body_id.x * 6 + 4, 0);
        D_T.append(index_mult + 5, body_id.x * 6 + 5, 0);
      }
      if (contact_active_pairs[index].y) {
        D_T.append(index_mult + 5, body_id.y * 6 + 3, 0);
        D_T.append(index_mult + 5, body_id.y * 6 + 4, 0);
        D_T.append(index_mult + 5, body_id.y * 6 + 5, 0);
      }
      D_T.finalize(index_mult + 5);
    }
  }
}
