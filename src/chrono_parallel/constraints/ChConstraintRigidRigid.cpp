#include <algorithm>
#include <limits>

#include "chrono_parallel/ChConfigParallel.h"
#include "chrono_parallel/constraints/ChConstraintRigidRigid.h"
#include "chrono_parallel/constraints/ChConstraintUtils.h"
//#include "chrono_parallel/math/quartic.h"

#include <thrust/iterator/constant_iterator.h>

using namespace chrono;

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

void ChConstraintRigidRigid::func_Project_normal(int index, const vec2* ids, const real* cohesion, real* gamma) {
    real gamma_x = gamma[index * 1 + 0];
    vec2 body_id = ids[index];
    real coh = cohesion[index];

    gamma_x += coh;
    gamma_x = gamma_x < 0 ? 0 : gamma_x - coh;
    gamma[index * 1 + 0] = gamma_x;
    if (data_manager->settings.solver.solver_mode == SolverMode::SLIDING) {
        gamma[data_manager->num_rigid_contacts + index * 2 + 0] = 0;
        gamma[data_manager->num_rigid_contacts + index * 2 + 1] = 0;
    }
    if (data_manager->settings.solver.solver_mode == SolverMode::SPINNING) {
        gamma[3 * data_manager->num_rigid_contacts + index * 3 + 0] = 0;
        gamma[3 * data_manager->num_rigid_contacts + index * 3 + 1] = 0;
        gamma[3 * data_manager->num_rigid_contacts + index * 3 + 2] = 0;
    }
}

void ChConstraintRigidRigid::func_Project_sliding(int index,
                                                  const vec2* ids,
                                                  const real3* fric,
                                                  const real* cohesion,
                                                  real* gam) {
    real3 gamma;
    gamma.x = gam[index * 1 + 0];
    gamma.y = gam[data_manager->num_rigid_contacts + index * 2 + 0];
    gamma.z = gam[data_manager->num_rigid_contacts + index * 2 + 1];

    //  if (data_manager->settings.solver.solver_mode == SPINNING) {
    //    gam[3 * data_manager->num_contacts + index * 3 + 0] = 0;
    //    gam[3 * data_manager->num_contacts + index * 3 + 1] = 0;
    //    gam[3 * data_manager->num_contacts + index * 3 + 2] = 0;
    //  }

    real coh = cohesion[index];
    gamma.x += coh;

    real mu = fric[index].x;
    if (mu == 0) {
        gamma.x = gamma.x < 0 ? 0 : gamma.x - coh;
        gamma.y = gamma.z = 0;

        gam[index * 1 + 0] = gamma.x;
        gam[data_manager->num_rigid_contacts + index * 2 + 0] = gamma.y;
        gam[data_manager->num_rigid_contacts + index * 2 + 1] = gamma.z;

        return;
    }

    if (Cone_generalized(gamma.x, gamma.y, gamma.z, mu)) {
    }

    gam[index * 1 + 0] = gamma.x - coh;
    gam[data_manager->num_rigid_contacts + index * 2 + 0] = gamma.y;
    gam[data_manager->num_rigid_contacts + index * 2 + 1] = gamma.z;
}
void ChConstraintRigidRigid::func_Project_spinning(int index, const vec2* ids, const real3* fric, real* gam) {
    // real3 gamma_roll = R3(0);
    real rollingfriction = fric[index].y;
    real spinningfriction = fric[index].z;

    //	if(rollingfriction||spinningfriction){
    //		gam[index + number_of_contacts * 1] = 0;
    //		gam[index + number_of_contacts * 2] = 0;
    //	}

    real gamma_n = gam[index * 1 + 0];
    real gamma_s = gam[3 * data_manager->num_rigid_contacts + index * 3 + 0];
    real gamma_tu = gam[3 * data_manager->num_rigid_contacts + index * 3 + 1];
    real gamma_tv = gam[3 * data_manager->num_rigid_contacts + index * 3 + 2];

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
    gam[index * 1 + 0] = gamma_n;
    gam[3 * data_manager->num_rigid_contacts + index * 3 + 0] = gamma_s;
    gam[3 * data_manager->num_rigid_contacts + index * 3 + 1] = gamma_tu;
    gam[3 * data_manager->num_rigid_contacts + index * 3 + 2] = gamma_tv;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ChConstraintRigidRigid::host_Project_single(int index, vec2* ids, real3* friction, real* cohesion, real* gamma) {
    // always project normal
    switch (data_manager->settings.solver.local_solver_mode) {
        case SolverMode::NORMAL: {
            func_Project_normal(index, ids, cohesion, gamma);
        } break;

        case SolverMode::SLIDING: {
            func_Project_sliding(index, ids, friction, cohesion, gamma);
        } break;

        case SolverMode::SPINNING: {
            func_Project_sliding(index, ids, friction, cohesion, gamma);
            func_Project_spinning(index, ids, friction, gamma);
        } break;
        default:
            break;
    }
}

void ChConstraintRigidRigid::Project(real* gamma) {
    const custom_vector<vec2>& bids = data_manager->host_data.bids_rigid_rigid;
    const custom_vector<real3>& friction = data_manager->host_data.fric_rigid_rigid;
    const custom_vector<real>& cohesion = data_manager->host_data.coh_rigid_rigid;

    switch (data_manager->settings.solver.local_solver_mode) {
        case SolverMode::NORMAL: {
#pragma omp parallel for
            for (int index = 0; index < (signed)data_manager->num_rigid_contacts; index++) {
                func_Project_normal(index, bids.data(), cohesion.data(), gamma);
            }
        } break;

        case SolverMode::SLIDING: {
#pragma omp parallel for
            for (int index = 0; index < (signed)data_manager->num_rigid_contacts; index++) {
                func_Project_sliding(index, bids.data(), friction.data(), cohesion.data(), gamma);
            }
        } break;

        case SolverMode::SPINNING: {
#pragma omp parallel for
            for (int index = 0; index < (signed)data_manager->num_rigid_contacts; index++) {
                func_Project_sliding(index, bids.data(), friction.data(), cohesion.data(), gamma);
                func_Project_spinning(index, bids.data(), friction.data(), gamma);
            }
        } break;
        default:
            break;
    }
}
void ChConstraintRigidRigid::Project_Single(int index, real* gamma) {
    custom_vector<vec2>& bids = data_manager->host_data.bids_rigid_rigid;
    custom_vector<real3>& friction = data_manager->host_data.fric_rigid_rigid;
    custom_vector<real>& cohesion = data_manager->host_data.coh_rigid_rigid;

    // host_Project_single(index, bids.data(), friction.data(), cohesion.data(), gamma);

    switch (data_manager->settings.solver.local_solver_mode) {
        case SolverMode::NORMAL: {
            func_Project_normal(index, bids.data(), cohesion.data(), gamma);
        } break;
        case SolverMode::SLIDING: {
            func_Project_sliding(index, bids.data(), friction.data(), cohesion.data(), gamma);
        } break;

        case SolverMode::SPINNING: {
            func_Project_sliding(index, bids.data(), friction.data(), cohesion.data(), gamma);
            func_Project_spinning(index, bids.data(), friction.data(), gamma);
        } break;
        default:
            break;
    }
}

void ChConstraintRigidRigid::Build_b() {
    if (data_manager->num_rigid_contacts <= 0) {
        return;
    }

#pragma omp parallel for
    for (int index = 0; index < (signed)data_manager->num_rigid_contacts; index++) {
        real bi = 0;
        real depth = data_manager->host_data.dpth_rigid_rigid[index];

        if (data_manager->settings.solver.alpha > 0) {
            bi = inv_hpa * depth;
        } else if (data_manager->settings.solver.contact_recovery_speed < 0) {
            bi = inv_h * depth;
        } else {
            bi = std::max(inv_h * depth, -data_manager->settings.solver.contact_recovery_speed);
            // bi = std::min(bi, inv_h * data_manager->settings.solver.cohesion_epsilon);
        }

        data_manager->host_data.b[index * 1 + 0] = bi;
    }
}

void ChConstraintRigidRigid::Build_s() {
    if (data_manager->num_rigid_contacts <= 0) {
        return;
    }

    if (data_manager->settings.solver.solver_mode == SolverMode::NORMAL) {
        return;
    }

    vec2* ids = data_manager->host_data.bids_rigid_rigid.data();
    const SubMatrixType& D_t_T = _DTT_;
    DynamicVector<real> v_new;

    const DynamicVector<real>& M_invk = data_manager->host_data.M_invk;
    const DynamicVector<real>& gamma = data_manager->host_data.gamma;

    const SubMatrixType& M_invD_n = _MINVDN_;
    const SubMatrixType& M_invD_t = _MINVDT_;
    const SubMatrixType& M_invD_s = _MINVDS_;
    const SubMatrixType& M_invD_b = _MINVDB_;
    const CompressedMatrix<real, blaze::columnMajor>& M_invD = data_manager->host_data.M_invD;

    uint num_contacts = data_manager->num_rigid_contacts;
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;

    ConstSubVectorType gamma_b = subvector(gamma, num_unilaterals, num_bilaterals);
    ConstSubVectorType gamma_n = subvector(gamma, 0, num_contacts);

    v_new = M_invk + M_invD * gamma;

#pragma omp parallel for
    for (int index = 0; index < (signed)data_manager->num_rigid_contacts; index++) {
        real fric = data_manager->host_data.fric_rigid_rigid[index].x;
        vec2 body_id = ids[index];

        real s_v = D_t_T(index * 2 + 0, body_id.x * 6 + 0) * +v_new[body_id.x * 6 + 0] +
                   D_t_T(index * 2 + 0, body_id.x * 6 + 1) * +v_new[body_id.x * 6 + 1] +
                   D_t_T(index * 2 + 0, body_id.x * 6 + 2) * +v_new[body_id.x * 6 + 2] +
                   D_t_T(index * 2 + 0, body_id.x * 6 + 3) * +v_new[body_id.x * 6 + 3] +
                   D_t_T(index * 2 + 0, body_id.x * 6 + 4) * +v_new[body_id.x * 6 + 4] +
                   D_t_T(index * 2 + 0, body_id.x * 6 + 5) * +v_new[body_id.x * 6 + 5] +

                   D_t_T(index * 2 + 0, body_id.y * 6 + 0) * +v_new[body_id.y * 6 + 0] +
                   D_t_T(index * 2 + 0, body_id.y * 6 + 1) * +v_new[body_id.y * 6 + 1] +
                   D_t_T(index * 2 + 0, body_id.y * 6 + 2) * +v_new[body_id.y * 6 + 2] +
                   D_t_T(index * 2 + 0, body_id.y * 6 + 3) * +v_new[body_id.y * 6 + 3] +
                   D_t_T(index * 2 + 0, body_id.y * 6 + 4) * +v_new[body_id.y * 6 + 4] +
                   D_t_T(index * 2 + 0, body_id.y * 6 + 5) * +v_new[body_id.y * 6 + 5];

        real s_w = D_t_T(index * 2 + 1, body_id.x * 6 + 0) * +v_new[body_id.x * 6 + 0] +
                   D_t_T(index * 2 + 1, body_id.x * 6 + 1) * +v_new[body_id.x * 6 + 1] +
                   D_t_T(index * 2 + 1, body_id.x * 6 + 2) * +v_new[body_id.x * 6 + 2] +
                   D_t_T(index * 2 + 1, body_id.x * 6 + 3) * +v_new[body_id.x * 6 + 3] +
                   D_t_T(index * 2 + 1, body_id.x * 6 + 4) * +v_new[body_id.x * 6 + 4] +
                   D_t_T(index * 2 + 1, body_id.x * 6 + 5) * +v_new[body_id.x * 6 + 5] +

                   D_t_T(index * 2 + 1, body_id.y * 6 + 0) * +v_new[body_id.y * 6 + 0] +
                   D_t_T(index * 2 + 1, body_id.y * 6 + 1) * +v_new[body_id.y * 6 + 1] +
                   D_t_T(index * 2 + 1, body_id.y * 6 + 2) * +v_new[body_id.y * 6 + 2] +
                   D_t_T(index * 2 + 1, body_id.y * 6 + 3) * +v_new[body_id.y * 6 + 3] +
                   D_t_T(index * 2 + 1, body_id.y * 6 + 4) * +v_new[body_id.y * 6 + 4] +
                   D_t_T(index * 2 + 1, body_id.y * 6 + 5) * +v_new[body_id.y * 6 + 5];

        data_manager->host_data.s[index * 1 + 0] = sqrt(s_v * s_v + s_w * s_w) * fric;
    }
}

void ChConstraintRigidRigid::Build_E() {
    if (data_manager->num_rigid_contacts <= 0) {
        return;
    }
    SolverMode solver_mode = data_manager->settings.solver.solver_mode;
    DynamicVector<real>& E = data_manager->host_data.E;
    uint num_contacts = data_manager->num_rigid_contacts;

#pragma omp parallel for
    for (int index = 0; index < (signed)data_manager->num_rigid_contacts; index++) {
        vec2 body = data_manager->host_data.bids_rigid_rigid[index];

        real4 cA = data_manager->host_data.compliance_data[body.x];
        real4 cB = data_manager->host_data.compliance_data[body.y];

        real compliance_normal = (cA.x == 0 || cB.x == 0) ? 0 : (cA.x + cB.x) * .5;
        real compliance_sliding = (cA.y == 0 || cB.y == 0) ? 0 : (cA.y + cB.y) * .5;
        real compliance_spinning = (cA.z == 0 || cB.z == 0) ? 0 : (cA.z + cB.z) * .5;
        real compliance_rolling = (cA.w == 0 || cB.w == 0) ? 0 : (cA.w + cB.w) * .5;

        E[index * 1 + 0] = inv_hhpa * compliance_normal;
        if (solver_mode == SolverMode::SLIDING) {
            E[num_contacts + index * 2 + 0] = inv_hhpa * compliance_sliding;
            E[num_contacts + index * 2 + 1] = inv_hhpa * compliance_sliding;
        } else if (solver_mode == SolverMode::SPINNING) {
            E[3 * num_contacts + index * 3 + 0] = inv_hhpa * compliance_spinning;
            E[3 * num_contacts + index * 3 + 1] = inv_hhpa * compliance_rolling;
            E[3 * num_contacts + index * 3 + 2] = inv_hhpa * compliance_rolling;
        }
    }
}

void ChConstraintRigidRigid::Build_D() {
    LOG(INFO) << "ChConstraintRigidRigid::Build_D";
    real3* norm = data_manager->host_data.norm_rigid_rigid.data();
    real3* ptA = data_manager->host_data.cpta_rigid_rigid.data();
    real3* ptB = data_manager->host_data.cptb_rigid_rigid.data();
    real3* pos_data = data_manager->host_data.pos_rigid.data();
    vec2* ids = data_manager->host_data.bids_rigid_rigid.data();
    quaternion* rot = data_manager->host_data.rot_rigid.data();

    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    const CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;

    SolverMode solver_mode = data_manager->settings.solver.solver_mode;

    const std::vector<std::shared_ptr<ChBody>>* body_list = data_manager->body_list;

#pragma omp parallel for
    for (int index = 0; index < (signed)data_manager->num_rigid_contacts; index++) {
        real3 U = norm[index], V, W;
        real3 T3, T4, T5, T6, T7, T8;
        real3 TA, TB, TC;
        real3 TD, TE, TF;
        Orthogonalize(U, V, W);
        vec2 body_id = ids[index];
        int off = 0;
        int row = index;
        // The position is subtracted here now instead of performing it in the narrowphase

        // Normal jacobian entries
        real3_int sbar_a = rotated_point_a[index];
        real3_int sbar_b = rotated_point_b[index];
        body_id.x = sbar_a.i;
        body_id.y = sbar_b.i;
        quaternion q_a = quat_a[index];
        quaternion q_b = quat_b[index];

        NORMAL_J

        SetRow6Check(D_T, off + row * 1 + 0, body_id.x * 6, -U, T3);
        SetRow6Check(D_T, off + row * 1 + 0, body_id.y * 6, U, -T6);

        if (solver_mode == SolverMode::SLIDING || solver_mode == SolverMode::SPINNING) {
            off = data_manager->num_rigid_contacts;

            SLIDING_J

            SetRow6Check(D_T, off + row * 2 + 0, body_id.x * 6, -V, T4);
            SetRow6Check(D_T, off + row * 2 + 1, body_id.x * 6, -W, T5);

            SetRow6Check(D_T, off + row * 2 + 0, body_id.y * 6, V, -T7);
            SetRow6Check(D_T, off + row * 2 + 1, body_id.y * 6, W, -T8);

            if (solver_mode == SolverMode::SPINNING) {
                off = 3 * data_manager->num_rigid_contacts;

                SetRow3Check(D_T, off + row * 3 + 0, body_id.x * 6 + 3, -U_A);
                SetRow3Check(D_T, off + row * 3 + 1, body_id.x * 6 + 3, -V_A);
                SetRow3Check(D_T, off + row * 3 + 2, body_id.x * 6 + 3, -W_A);

                SetRow3Check(D_T, off + row * 3 + 0, body_id.y * 6 + 3, U_B);
                SetRow3Check(D_T, off + row * 3 + 1, body_id.y * 6 + 3, V_B);
                SetRow3Check(D_T, off + row * 3 + 2, body_id.y * 6 + 3, W_B);
            }
        }
    }
}

void ChConstraintRigidRigid::GenerateSparsity() {
    LOG(INFO) << "ChConstraintRigidRigid::GenerateSparsity";
    SolverMode solver_mode = data_manager->settings.solver.solver_mode;

    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    const vec2* ids = data_manager->host_data.bids_rigid_rigid.data();

    for (int index = 0; index < (signed)data_manager->num_rigid_contacts; index++) {
        vec2 body_id = ids[index];
        int row = index;
        int off = 0;

        AppendRow6(D_T, off + row * 1, body_id.x * 6, 0);
        AppendRow6(D_T, off + row * 1, body_id.y * 6, 0);

        D_T.finalize(off + row * 1 + 0);
    }

    if (solver_mode == SolverMode::SLIDING || solver_mode == SolverMode::SPINNING) {
        for (int index = 0; index < (signed)data_manager->num_rigid_contacts; index++) {
            vec2 body_id = ids[index];
            int row = index;
            int off = data_manager->num_rigid_contacts;

            AppendRow6(D_T, off + row * 2 + 0, body_id.x * 6, 0);
            AppendRow6(D_T, off + row * 2 + 0, body_id.y * 6, 0);

            D_T.finalize(off + row * 2 + 0);

            AppendRow6(D_T, off + row * 2 + 1, body_id.x * 6, 0);
            AppendRow6(D_T, off + row * 2 + 1, body_id.y * 6, 0);

            D_T.finalize(off + row * 2 + 1);
        }
    }

    if (solver_mode == SolverMode::SPINNING) {
        for (int index = 0; index < (signed)data_manager->num_rigid_contacts; index++) {
            vec2 body_id = ids[index];
            int row = index;
            int off = 3 * data_manager->num_rigid_contacts;
            D_T.append(off + row * 3 + 0, body_id.x * 6 + 3, 0);
            D_T.append(off + row * 3 + 0, body_id.x * 6 + 4, 0);
            D_T.append(off + row * 3 + 0, body_id.x * 6 + 5, 0);

            D_T.append(off + row * 3 + 0, body_id.y * 6 + 3, 0);
            D_T.append(off + row * 3 + 0, body_id.y * 6 + 4, 0);
            D_T.append(off + row * 3 + 0, body_id.y * 6 + 5, 0);

            D_T.finalize(off + row * 3 + 0);

            D_T.append(off + row * 3 + 1, body_id.x * 6 + 3, 0);
            D_T.append(off + row * 3 + 1, body_id.x * 6 + 4, 0);
            D_T.append(off + row * 3 + 1, body_id.x * 6 + 5, 0);

            D_T.append(off + row * 3 + 1, body_id.y * 6 + 3, 0);
            D_T.append(off + row * 3 + 1, body_id.y * 6 + 4, 0);
            D_T.append(off + row * 3 + 1, body_id.y * 6 + 5, 0);

            D_T.finalize(off + row * 3 + 1);

            D_T.append(off + row * 3 + 2, body_id.x * 6 + 3, 0);
            D_T.append(off + row * 3 + 2, body_id.x * 6 + 4, 0);
            D_T.append(off + row * 3 + 2, body_id.x * 6 + 5, 0);

            D_T.append(off + row * 3 + 2, body_id.y * 6 + 3, 0);
            D_T.append(off + row * 3 + 2, body_id.y * 6 + 4, 0);
            D_T.append(off + row * 3 + 2, body_id.y * 6 + 5, 0);

            D_T.finalize(off + row * 3 + 2);
        }
    }
}

void ChConstraintRigidRigid::Dx(const DynamicVector<real>& gam, DynamicVector<real>& XYZUVW) {
    const int& num_rigid_contacts = data_manager->num_rigid_contacts;
    custom_vector<char>& active_rigid = data_manager->host_data.active_rigid;
    real3* norm = data_manager->host_data.norm_rigid_rigid.data();
    real3* ptA = data_manager->host_data.cpta_rigid_rigid.data();
    real3* ptB = data_manager->host_data.cptb_rigid_rigid.data();
    real3* pos = data_manager->host_data.pos_rigid.data();
    vec2* bids_rigid_rigid = data_manager->host_data.bids_rigid_rigid.data();
    quaternion* rot = data_manager->host_data.rot_rigid.data();

#pragma omp parallel for
    for (int i = 0; i < num_rigid_contacts; i++) {
        real3 U = real3(norm[i]), V, W;
        Orthogonalize(U, V, W);
        // int id_a = bids_rigid_rigid[i].x;
        // int id_b = bids_rigid_rigid[i].y;

        real3 T3, T4, T5, T6, T7, T8;
        real3 g = real3(gam[i], gam[num_rigid_contacts + i * 2 + 0], gam[num_rigid_contacts + i * 2 + 1]);
        {
            real3_int sbar = rotated_point_a[i];
            quaternion q_a = quat_a[i];
            T3 = Cross(Rotate(U, q_a), sbar.v);
            T4 = Cross(Rotate(V, q_a), sbar.v);
            T5 = Cross(Rotate(W, q_a), sbar.v);

            // if (active_rigid[id_a] != 0)

            real3 res = -U * g.x - V * g.y - W * g.z;

#pragma omp atomic
            XYZUVW[sbar.i * 6 + 0] += res.x;
#pragma omp atomic
            XYZUVW[sbar.i * 6 + 1] += res.y;
#pragma omp atomic
            XYZUVW[sbar.i * 6 + 2] += res.z;

            res = T3 * g.x + T4 * g.y + T5 * g.z;

#pragma omp atomic
            XYZUVW[sbar.i * 6 + 3] += res.x;
#pragma omp atomic
            XYZUVW[sbar.i * 6 + 4] += res.y;
#pragma omp atomic
            XYZUVW[sbar.i * 6 + 5] += res.z;
        }

        {
            real3_int sbar = rotated_point_b[i];
            quaternion q_b = quat_b[i];
            T6 = Cross(Rotate(U, q_b), sbar.v);
            T7 = Cross(Rotate(V, q_b), sbar.v);
            T8 = Cross(Rotate(W, q_b), sbar.v);

            // if (active_rigid[id_b] != 0)

            real3 res = U * g.x + V * g.y + W * g.z;

#pragma omp atomic
            XYZUVW[sbar.i * 6 + 0] += res.x;
#pragma omp atomic
            XYZUVW[sbar.i * 6 + 1] += res.y;
#pragma omp atomic
            XYZUVW[sbar.i * 6 + 2] += res.z;

            res = -T6 * g.x - T7 * g.y - T8 * g.z;

#pragma omp atomic
            XYZUVW[sbar.i * 6 + 3] += res.x;
#pragma omp atomic
            XYZUVW[sbar.i * 6 + 4] += res.y;
#pragma omp atomic
            XYZUVW[sbar.i * 6 + 5] += res.z;
        }
    }
}

void ChConstraintRigidRigid::D_Tx(const DynamicVector<real>& XYZUVW, DynamicVector<real>& out_vector) {
    const int& num_rigid_contacts = data_manager->num_rigid_contacts;
    custom_vector<char>& active_rigid = data_manager->host_data.active_rigid;
    real3* norm = data_manager->host_data.norm_rigid_rigid.data();
    real3* ptA = data_manager->host_data.cpta_rigid_rigid.data();
    real3* ptB = data_manager->host_data.cptb_rigid_rigid.data();
    real3* pos = data_manager->host_data.pos_rigid.data();
    vec2* bids_rigid_rigid = data_manager->host_data.bids_rigid_rigid.data();
    quaternion* rot = data_manager->host_data.rot_rigid.data();

#pragma omp parallel for
    for (int i = 0; i < num_rigid_contacts; i++) {
        real3 U = real3(norm[i]), V, W;
        Orthogonalize(U, V, W);
        real temp[3] = {0, 0, 0};

        // real3 pA = ptA[i];
        // int id_a = bids_rigid_rigid[i].x;
        // if (active_rigid[id_a] != 0)
        {
            real3_int sbar = rotated_point_a[i];
            quaternion quaternion_conjugate = quat_a[i];

            real3 XYZ(XYZUVW[sbar.i * 6 + 0], XYZUVW[sbar.i * 6 + 1], XYZUVW[sbar.i * 6 + 2]);
            real3 UVW(XYZUVW[sbar.i * 6 + 3], XYZUVW[sbar.i * 6 + 4], XYZUVW[sbar.i * 6 + 5]);

            real3 T1 = Cross(Rotate(U, quaternion_conjugate), sbar.v);
            real3 T2 = Cross(Rotate(V, quaternion_conjugate), sbar.v);
            real3 T3 = Cross(Rotate(W, quaternion_conjugate), sbar.v);

            temp[0] = Dot(XYZ, -U) + Dot(UVW, T1);
            temp[1] = Dot(XYZ, -V) + Dot(UVW, T2);
            temp[2] = Dot(XYZ, -W) + Dot(UVW, T3);

            // Jacobian<false>(rot[id_a], U, V, W, ptA[i] - pos[id_a], &XYZUVW[id_a * 6 + 0], temp);
        }

        // real3 pB = ptB[i];
        // int id_b = bids_rigid_rigid[i].y;
        // if (active_rigid[id_b] != 0)
        {
            real3_int sbar = rotated_point_b[i];
            quaternion quaternion_conjugate = quat_b[i];

            real3 XYZ(XYZUVW[sbar.i * 6 + 0], XYZUVW[sbar.i * 6 + 1], XYZUVW[sbar.i * 6 + 2]);
            real3 UVW(XYZUVW[sbar.i * 6 + 3], XYZUVW[sbar.i * 6 + 4], XYZUVW[sbar.i * 6 + 5]);

            real3 T1 = Cross(Rotate(U, quaternion_conjugate), sbar.v);
            real3 T2 = Cross(Rotate(V, quaternion_conjugate), sbar.v);
            real3 T3 = Cross(Rotate(W, quaternion_conjugate), sbar.v);

            temp[0] += Dot(XYZ, U) + Dot(UVW, -T1);
            temp[1] += Dot(XYZ, V) + Dot(UVW, -T2);
            temp[2] += Dot(XYZ, W) + Dot(UVW, -T3);
        }

        out_vector[i] = temp[0];
        out_vector[num_rigid_contacts + i * 2 + 0] = temp[1];
        out_vector[num_rigid_contacts + i * 2 + 1] = temp[2];

        //            out_vector[CONTACT + 3] = temp[3];
        //            out_vector[CONTACT + 4] = temp[4];
        //            out_vector[CONTACT + 5] = temp[5];
    }

    //    // data_manager->PrintMatrix(data_manager->host_data.D);

    //    DynamicVector<real> compare =
    //            data_manager->host_data.D_T * data_manager->host_data.D * data_manager->host_data.gamma;
    //    std::cout << "nconstr " << compare.size() << std::endl;
    //    for (int i = 0; i < compare.size(); i++) {
    //        std::cout << compare[i] << " " << out_vector[i] << std::endl;
    //    }
}
