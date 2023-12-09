// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Handling of rigid contact and calculation of corrections and Jacobians
//
// =============================================================================

#include <algorithm>
#include <limits>

#include "chrono_multicore/ChConfigMulticore.h"
#include "chrono_multicore/constraints/ChConstraintRigidRigid.h"
#include "chrono_multicore/constraints/ChConstraintUtils.h"

#include <thrust/iterator/constant_iterator.h>

using namespace chrono;

// -----------------------------------------------------------------------------

ChConstraintRigidRigid::ChConstraintRigidRigid()
    : data_manager(nullptr), offset(3), inv_h(0), inv_hpa(0), inv_hhpa(0) {}

void ChConstraintRigidRigid::func_Project_normal(int index, const vec2* ids, const real* cohesion, real* gamma) {
    const auto num_rigid_contacts = data_manager->cd_data ? data_manager->cd_data->num_rigid_contacts : 0;

    real gamma_x = gamma[index * 1 + 0];
    real coh = cohesion[index];

    gamma[index * 1 + 0] = (gamma_x < -coh) ? -coh : gamma_x;
    switch (data_manager->settings.solver.solver_mode) {
        case SolverMode::SLIDING:
            gamma[num_rigid_contacts + index * 2 + 0] = 0;
            gamma[num_rigid_contacts + index * 2 + 1] = 0;
            break;
        case SolverMode::SPINNING:
            gamma[num_rigid_contacts + index * 2 + 0] = 0;
            gamma[num_rigid_contacts + index * 2 + 1] = 0;
            gamma[3 * num_rigid_contacts + index * 3 + 0] = 0;
            gamma[3 * num_rigid_contacts + index * 3 + 1] = 0;
            gamma[3 * num_rigid_contacts + index * 3 + 2] = 0;
            break;
        default:
            break;
    }
}

void ChConstraintRigidRigid::func_Project_sliding(int index,
                                                  const vec2* ids,
                                                  const real3* fric,
                                                  const real* cohesion,
                                                  real* gam) {
    const auto num_rigid_contacts = data_manager->cd_data->num_rigid_contacts;

    real3 gamma;
    gamma.x = gam[index * 1 + 0];
    gamma.y = gam[num_rigid_contacts + index * 2 + 0];
    gamma.z = gam[num_rigid_contacts + index * 2 + 1];

    real coh = cohesion[index];
    real mu = fric[index].x;

    if (mu == 0) {
        gam[index * 1 + 0] = (gamma.x < -coh) ? -coh : gamma.x;
        gam[num_rigid_contacts + index * 2 + 0] = 0;
        gam[num_rigid_contacts + index * 2 + 1] = 0;
        return;
    }

    gamma.x += coh;
    Cone_generalized_rigid(gamma.x, gamma.y, gamma.z, mu);
    gam[index * 1 + 0] = gamma.x - coh;
    gam[num_rigid_contacts + index * 2 + 0] = gamma.y;
    gam[num_rigid_contacts + index * 2 + 1] = gamma.z;
}

void ChConstraintRigidRigid::func_Project_spinning(int index, const vec2* ids, const real3* fric, real* gam) {
    const auto num_rigid_contacts = data_manager->cd_data->num_rigid_contacts;

    real rollingfriction = fric[index].y;
    real spinningfriction = fric[index].z;

    real f_n = gam[index * 1 + 0];
    real t_n = gam[3 * num_rigid_contacts + index * 3 + 0];
    real t_u = gam[3 * num_rigid_contacts + index * 3 + 1];
    real t_v = gam[3 * num_rigid_contacts + index * 3 + 2];

    real t_tang = sqrt(t_v * t_v + t_u * t_u);
    real t_sptang = fabs(t_n);  // = sqrt(t_n*t_n);

    if (spinningfriction) {
        if (t_sptang < spinningfriction * f_n) {
            // inside upper cone? keep untouched!
        } else {
            // inside lower cone? reset  normal,u,v to zero!
            if ((t_sptang < -(1 / spinningfriction) * f_n) || (fabs(f_n) < 10e-15)) {
                gam[index * 1 + 0] = 0;
                gam[3 * num_rigid_contacts + index * 3 + 0] = 0;
            } else {
                // remaining case: project orthogonally to generator segment of upper cone (CAN BE simplified)
                real f_n_proj = (t_sptang * spinningfriction + f_n) / (spinningfriction * spinningfriction + 1);
                real t_tang_proj = f_n_proj * spinningfriction;
                real tproj_div_t = t_tang_proj / t_sptang;
                real t_n_proj = tproj_div_t * t_n;

                gam[index * 1 + 0] = f_n_proj;
                gam[3 * num_rigid_contacts + index * 3 + 0] = t_n_proj;
            }
        }
    }

    if (!rollingfriction) {
        gam[3 * num_rigid_contacts + index * 3 + 1] = 0;
        gam[3 * num_rigid_contacts + index * 3 + 2] = 0;

        if (f_n < 0)
            gam[index * 1 + 0] = 0;
        return;
    }
    if (t_tang < rollingfriction * f_n)
        return;

    if ((t_tang < -(1 / rollingfriction) * f_n) || (fabs(f_n) < 10e-15)) {
        real f_n_proj = 0;
        real t_u_proj = 0;
        real t_v_proj = 0;

        gam[index * 1 + 0] = f_n_proj;
        gam[3 * num_rigid_contacts + index * 3 + 1] = t_u_proj;
        gam[3 * num_rigid_contacts + index * 3 + 2] = t_v_proj;

        return;
    }
    real f_n_proj = (t_tang * rollingfriction + f_n) / (rollingfriction * rollingfriction + 1);
    real t_tang_proj = f_n_proj * rollingfriction;
    real tproj_div_t = t_tang_proj / t_tang;
    real t_u_proj = tproj_div_t * t_u;
    real t_v_proj = tproj_div_t * t_v;

    gam[index * 1 + 0] = f_n_proj;
    gam[3 * num_rigid_contacts + index * 3 + 1] = t_u_proj;
    gam[3 * num_rigid_contacts + index * 3 + 2] = t_v_proj;
}

// -----------------------------------------------------------------------------

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

void ChConstraintRigidRigid::Setup(ChMulticoreDataManager* dm) {
    data_manager = dm;
    const auto num_rigid_contacts = data_manager->cd_data ? data_manager->cd_data->num_rigid_contacts : 0;

    inv_h = 1 / data_manager->settings.step_size;
    inv_hpa = 1 / (data_manager->settings.step_size + data_manager->settings.solver.alpha);
    inv_hhpa = inv_h * inv_hpa;

    if (num_rigid_contacts <= 0) {
        return;
    }

    contact_active_pairs.resize(int(num_rigid_contacts));
    rotated_point_a.resize(num_rigid_contacts);
    rotated_point_b.resize(num_rigid_contacts);
    quat_a.resize(num_rigid_contacts);
    quat_b.resize(num_rigid_contacts);

    // Readability replacements
    auto& bids = data_manager->cd_data->bids_rigid_rigid;  // global IDs of bodies in contact
    auto& abody = data_manager->host_data.active_rigid;    // flags for active bodies

#pragma omp parallel for
    for (int i = 0; i < (signed)num_rigid_contacts; i++) {
        auto b1 = bids[i].x;  // global IDs of bodies in contact
        auto b2 = bids[i].y;  //

        contact_active_pairs[i] = bool2(abody[b1] != 0, abody[b2] != 0);

        {
            quaternion quaternion_conjugate = ~data_manager->host_data.rot_rigid[b1];
            real3 sbar = Rotate(data_manager->cd_data->cpta_rigid_rigid[i] - data_manager->host_data.pos_rigid[b1],
                                quaternion_conjugate);

            rotated_point_a[i] = real3_int(sbar, b1);
            quat_a[i] = quaternion_conjugate;
        }
        {
            quaternion quaternion_conjugate = ~data_manager->host_data.rot_rigid[b2];
            real3 sbar = Rotate(data_manager->cd_data->cptb_rigid_rigid[i] - data_manager->host_data.pos_rigid[b2],
                                quaternion_conjugate);

            rotated_point_b[i] = real3_int(sbar, b2);
            quat_b[i] = quaternion_conjugate;
        }
    }
}

void ChConstraintRigidRigid::Project(real* gamma) {
    const auto num_rigid_contacts = data_manager->cd_data ? data_manager->cd_data->num_rigid_contacts : 0;

    if (num_rigid_contacts <= 0)
        return;

    const custom_vector<vec2>& bids = data_manager->cd_data->bids_rigid_rigid;
    const custom_vector<real3>& friction = data_manager->host_data.fric_rigid_rigid;
    const custom_vector<real>& cohesion = data_manager->host_data.coh_rigid_rigid;

    switch (data_manager->settings.solver.local_solver_mode) {
        case SolverMode::NORMAL: {
#pragma omp parallel for
            for (int index = 0; index < (signed)num_rigid_contacts; index++) {
                func_Project_normal(index, bids.data(), cohesion.data(), gamma);
            }
        } break;

        case SolverMode::SLIDING: {
#pragma omp parallel for
            for (int index = 0; index < (signed)num_rigid_contacts; index++) {
                func_Project_sliding(index, bids.data(), friction.data(), cohesion.data(), gamma);
            }
        } break;

        case SolverMode::SPINNING: {
#pragma omp parallel for
            for (int index = 0; index < (signed)num_rigid_contacts; index++) {
                func_Project_sliding(index, bids.data(), friction.data(), cohesion.data(), gamma);
                func_Project_spinning(index, bids.data(), friction.data(), gamma);
            }
        } break;
        default:
            break;
    }
}

void ChConstraintRigidRigid::Project_Single(int index, real* gamma) {
    custom_vector<vec2>& bids = data_manager->cd_data->bids_rigid_rigid;
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
    const auto num_rigid_contacts = data_manager->cd_data ? data_manager->cd_data->num_rigid_contacts : 0;

    if (num_rigid_contacts <= 0) {
        return;
    }

#pragma omp parallel for
    for (int index = 0; index < (signed)num_rigid_contacts; index++) {
        real bi = 0;
        real depth = data_manager->cd_data->dpth_rigid_rigid[index];

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
    const auto num_rigid_contacts = data_manager->cd_data ? data_manager->cd_data->num_rigid_contacts : 0;

    if (num_rigid_contacts <= 0) {
        return;
    }

    if (data_manager->settings.solver.solver_mode == SolverMode::NORMAL) {
        return;
    }

    vec2* ids = data_manager->cd_data->bids_rigid_rigid.data();
    const SubMatrixType& D_t_T = _DTT_;
    DynamicVector<real> v_new;

    const DynamicVector<real>& M_invk = data_manager->host_data.M_invk;
    const DynamicVector<real>& gamma = data_manager->host_data.gamma;

    const CompressedMatrix<real, blaze::columnMajor>& M_invD = data_manager->host_data.M_invD;

    v_new = M_invk + M_invD * gamma;

#pragma omp parallel for
    for (int index = 0; index < (signed)num_rigid_contacts; index++) {
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
    const auto num_rigid_contacts = data_manager->cd_data ? data_manager->cd_data->num_rigid_contacts : 0;

    if (num_rigid_contacts <= 0)
        return;

    SolverMode solver_mode = data_manager->settings.solver.solver_mode;
    DynamicVector<real>& E = data_manager->host_data.E;
    uint num_contacts = num_rigid_contacts;
    const custom_vector<real4>& compliance = data_manager->host_data.compliance_rigid_rigid;

#pragma omp parallel for
    for (int index = 0; index < (signed)num_rigid_contacts; index++) {
        ////vec2 body = data_manager->cd_data->bids_rigid_rigid[index];

        real compliance_normal = compliance[index].x;
        real compliance_sliding = compliance[index].y;
        real compliance_rolling = compliance[index].z;
        real compliance_spinning = compliance[index].w;

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
    const auto num_rigid_contacts = data_manager->cd_data ? data_manager->cd_data->num_rigid_contacts : 0;

    if (num_rigid_contacts <= 0)
        return;

    real3* norm = data_manager->cd_data->norm_rigid_rigid.data();
    vec2* ids = data_manager->cd_data->bids_rigid_rigid.data();

    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    SolverMode solver_mode = data_manager->settings.solver.solver_mode;

#pragma omp parallel for
    for (int index = 0; index < (signed)num_rigid_contacts; index++) {
        const real3& U = norm[index];
        real3 V, W;
        Orthogonalize(U, V, W);
        vec2 body_id = ids[index];
        int off = 0;
        int row = index;
        // The position is subtracted here now instead of performing it in the narrowphase

        real3 T3, T4, T5, T6, T7, T8;
        real3 TA, TB, TC;
        real3 TD, TE, TF;

        // Normal jacobian entries
        const real3_int& sbar_a = rotated_point_a[index];
        const real3_int& sbar_b = rotated_point_b[index];
        body_id.x = sbar_a.i;
        body_id.y = sbar_b.i;
        const quaternion& q_a = quat_a[index];
        const quaternion& q_b = quat_b[index];

        real3 U_A = Rotate(U, q_a);
        T3 = Cross(U_A, sbar_a.v);
        real3 U_B = Rotate(U, q_b);
        T6 = Cross(U_B, sbar_b.v);

        SetRow6Check(D_T, off + row * 1 + 0, body_id.x * 6, -U, T3);
        SetRow6Check(D_T, off + row * 1 + 0, body_id.y * 6, U, -T6);

        if (solver_mode == SolverMode::SLIDING || solver_mode == SolverMode::SPINNING) {
            off = num_rigid_contacts;

            real3 V_A = Rotate(V, q_a);
            real3 W_A = Rotate(W, q_a);
            T4 = Cross(V_A, sbar_a.v);
            T5 = Cross(W_A, sbar_a.v);

            real3 V_B = Rotate(V, q_b);
            real3 W_B = Rotate(W, q_b);
            T7 = Cross(V_B, sbar_b.v);
            T8 = Cross(W_B, sbar_b.v);

            SetRow6Check(D_T, off + row * 2 + 0, body_id.x * 6, -V, T4);
            SetRow6Check(D_T, off + row * 2 + 1, body_id.x * 6, -W, T5);

            SetRow6Check(D_T, off + row * 2 + 0, body_id.y * 6, V, -T7);
            SetRow6Check(D_T, off + row * 2 + 1, body_id.y * 6, W, -T8);

            if (solver_mode == SolverMode::SPINNING) {
                off = 3 * num_rigid_contacts;

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
    const auto num_rigid_contacts = data_manager->cd_data ? data_manager->cd_data->num_rigid_contacts : 0;

    if (num_rigid_contacts <= 0)
        return;

    SolverMode solver_mode = data_manager->settings.solver.solver_mode;

    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    const vec2* ids = data_manager->cd_data->bids_rigid_rigid.data();

    for (int index = 0; index < (signed)num_rigid_contacts; index++) {
        const vec2& body_id = ids[index];
        int row = index;
        int off = 0;

        AppendRow6(D_T, off + row * 1, body_id.x * 6, 0);
        AppendRow6(D_T, off + row * 1, body_id.y * 6, 0);

        D_T.finalize(off + row * 1 + 0);
    }

    if (solver_mode == SolverMode::SLIDING || solver_mode == SolverMode::SPINNING) {
        for (int index = 0; index < (signed)num_rigid_contacts; index++) {
            const vec2& body_id = ids[index];
            int row = index;
            int off = num_rigid_contacts;

            AppendRow6(D_T, off + row * 2 + 0, body_id.x * 6, 0);
            AppendRow6(D_T, off + row * 2 + 0, body_id.y * 6, 0);

            D_T.finalize(off + row * 2 + 0);

            AppendRow6(D_T, off + row * 2 + 1, body_id.x * 6, 0);
            AppendRow6(D_T, off + row * 2 + 1, body_id.y * 6, 0);

            D_T.finalize(off + row * 2 + 1);
        }
    }

    if (solver_mode == SolverMode::SPINNING) {
        for (int index = 0; index < (signed)num_rigid_contacts; index++) {
            const vec2& body_id = ids[index];
            int row = index;
            int off = 3 * num_rigid_contacts;
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
    const auto num_rigid_contacts = data_manager->cd_data->num_rigid_contacts;
    real3* norm = data_manager->cd_data->norm_rigid_rigid.data();
    ////custom_vector<char>& active_rigid = data_manager->host_data.active_rigid;
    ////vec2* bids_rigid_rigid = data_manager->host_data.bids_rigid_rigid.data();

#pragma omp parallel for
    for (int i = 0; i < (signed)num_rigid_contacts; i++) {
        const real3& U = real3(norm[i]);
        real3 V, W;
        Orthogonalize(U, V, W);
        ////int id_a = bids_rigid_rigid[i].x;
        ////int id_b = bids_rigid_rigid[i].y;

        real3 T3, T4, T5, T6, T7, T8;
        real3 g = real3(gam[i], gam[num_rigid_contacts + i * 2 + 0], gam[num_rigid_contacts + i * 2 + 1]);
        {
            const real3_int& sbar = rotated_point_a[i];
            const quaternion& q_a = quat_a[i];
            T3 = Cross(Rotate(U, q_a), sbar.v);
            T4 = Cross(Rotate(V, q_a), sbar.v);
            T5 = Cross(Rotate(W, q_a), sbar.v);

            ////if (active_rigid[id_a] != 0)

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
            const real3_int& sbar = rotated_point_b[i];
            const quaternion& q_b = quat_b[i];
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
    const auto num_rigid_contacts = data_manager->cd_data->num_rigid_contacts;
    real3* norm = data_manager->cd_data->norm_rigid_rigid.data();
    ////custom_vector<char>& active_rigid = data_manager->host_data.active_rigid;
    ////real3* ptA = data_manager->cd_data->cpta_rigid_rigid.data();
    ////real3* ptB = data_manager->cd_data->cptb_rigid_rigid.data();
    ////vec2* bids_rigid_rigid = data_manager->cd_data->bids_rigid_rigid.data();
    ////real3* pos = data_manager->host_data.pos_rigid.data();
    ////quaternion* rot = data_manager->host_data.rot_rigid.data();

#pragma omp parallel for
    for (int i = 0; i < (signed)num_rigid_contacts; i++) {
        const real3& U = real3(norm[i]);
        real3 V, W;
        Orthogonalize(U, V, W);
        real temp[3] = {0, 0, 0};

        ////real3 pA = ptA[i];
        ////int id_a = bids_rigid_rigid[i].x;
        ////if (active_rigid[id_a] != 0)
        {
            const real3_int& sbar = rotated_point_a[i];
            const quaternion& quaternion_conjugate = quat_a[i];

            real3 XYZ(XYZUVW[sbar.i * 6 + 0], XYZUVW[sbar.i * 6 + 1], XYZUVW[sbar.i * 6 + 2]);
            real3 UVW(XYZUVW[sbar.i * 6 + 3], XYZUVW[sbar.i * 6 + 4], XYZUVW[sbar.i * 6 + 5]);

            real3 T1 = Cross(Rotate(U, quaternion_conjugate), sbar.v);
            real3 T2 = Cross(Rotate(V, quaternion_conjugate), sbar.v);
            real3 T3 = Cross(Rotate(W, quaternion_conjugate), sbar.v);

            temp[0] = Dot(XYZ, -U) + Dot(UVW, T1);
            temp[1] = Dot(XYZ, -V) + Dot(UVW, T2);
            temp[2] = Dot(XYZ, -W) + Dot(UVW, T3);

            ////Jacobian<false>(rot[id_a], U, V, W, ptA[i] - pos[id_a], &XYZUVW[id_a * 6 + 0], temp);
        }

        ////real3 pB = ptB[i];
        ////int id_b = bids_rigid_rigid[i].y;
        ////if (active_rigid[id_b] != 0)
        {
            const real3_int& sbar = rotated_point_b[i];
            const quaternion& quaternion_conjugate = quat_b[i];

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
