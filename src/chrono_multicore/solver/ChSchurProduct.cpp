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
// Authors: Hammad Mazhar
// =============================================================================

#include "chrono_multicore/solver/ChSolverMulticore.h"

using namespace chrono;

ChSchurProduct::ChSchurProduct() {
    data_manager = 0;
}
void ChSchurProduct::operator()(const VectorType& x, VectorType& output) {
    data_manager->system_timer.start("SchurProduct");

    const VectorType& E = data_manager->host_data.E;

    uint num_rigid_contacts = data_manager->cd_data ? data_manager->cd_data->num_rigid_contacts : 0;
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;
    output.setZero();

    const SparseMatrixType& D_T = data_manager->host_data.D_T;
    const SparseMatrixType& Nschur = data_manager->host_data.Nschur;

    if (data_manager->settings.solver.local_solver_mode == data_manager->settings.solver.solver_mode) {
        if (data_manager->settings.solver.compute_N) {
            output = Nschur * x + E.cwiseProduct(x);
        } else {
            output = D_T * data_manager->host_data.M_invD * x + E.cwiseProduct(x);
        }

    } else {
        const SparseMatrixType& D_n_T = _DNT_;
        const SparseMatrixType& D_b_T = _DBT_;
        const SparseMatrixType& M_invD_n = _MINVDN_;
        const SparseMatrixType& M_invD_b = _MINVDB_;

        SubVectorType o_b = output.segment(num_unilaterals, num_bilaterals);
        ConstSubVectorType x_b = x.segment(num_unilaterals, num_bilaterals);
        ConstSubVectorType E_b = E.segment(num_unilaterals, num_bilaterals);

        SubVectorType o_n = output.segment(0, num_rigid_contacts);
        ConstSubVectorType x_n = x.segment(0, num_rigid_contacts);
        ConstSubVectorType E_n = E.segment(0, num_rigid_contacts);

        switch (data_manager->settings.solver.local_solver_mode) {
            case SolverMode::BILATERAL: {
                o_b = D_b_T * (M_invD_b * x_b) + E_b.cwiseProduct(x_b);
            } break;

            case SolverMode::NORMAL: {
                VectorType tmp = M_invD_b * x_b + M_invD_n * x_n;
                o_b = D_b_T * tmp + E_b.cwiseProduct(x_b);
                o_n = D_n_T * tmp + E_n.cwiseProduct(x_n);
            } break;

            case SolverMode::SLIDING: {
                const SparseMatrixType& D_t_T = _DTT_;
                const SparseMatrixType& M_invD_t = _MINVDT_;
                SubVectorType o_t = output.segment(num_rigid_contacts, num_rigid_contacts * 2);
                ConstSubVectorType x_t = x.segment(num_rigid_contacts, num_rigid_contacts * 2);
                ConstSubVectorType E_t = E.segment(num_rigid_contacts, num_rigid_contacts * 2);

                VectorType tmp = M_invD_b * x_b + M_invD_n * x_n + M_invD_t * x_t;
                o_b = D_b_T * tmp + E_b.cwiseProduct(x_b);
                o_n = D_n_T * tmp + E_n.cwiseProduct(x_n);
                o_t = D_t_T * tmp + E_t.cwiseProduct(x_t);

            } break;

            case SolverMode::SPINNING: {
                const SparseMatrixType& D_t_T = _DTT_;
                const SparseMatrixType& D_s_T = _DST_;
                const SparseMatrixType& M_invD_t = _MINVDT_;
                const SparseMatrixType& M_invD_s = _MINVDS_;
                SubVectorType o_t = output.segment(num_rigid_contacts, num_rigid_contacts * 2);
                ConstSubVectorType x_t = x.segment(num_rigid_contacts, num_rigid_contacts * 2);
                ConstSubVectorType E_t = E.segment(num_rigid_contacts, num_rigid_contacts * 2);

                SubVectorType o_s = output.segment(num_rigid_contacts * 3, num_rigid_contacts * 3);
                ConstSubVectorType x_s = x.segment(num_rigid_contacts * 3, num_rigid_contacts * 3);
                ConstSubVectorType E_s = E.segment(num_rigid_contacts * 3, num_rigid_contacts * 3);

                VectorType tmp = M_invD_b * x_b + M_invD_n * x_n + M_invD_t * x_t + M_invD_s * x_s;
                o_b = D_b_T * tmp + E_b.cwiseProduct(x_b);
                o_n = D_n_T * tmp + E_n.cwiseProduct(x_n);
                o_t = D_t_T * tmp + E_t.cwiseProduct(x_t);
                o_s = D_s_T * tmp + E_s.cwiseProduct(x_s);

            } break;
        }
    }
    data_manager->system_timer.stop("SchurProduct");
}

void ChSchurProductBilateral::Setup(ChMulticoreDataManager* data_container_) {
    ChSchurProduct::Setup(data_container_);
    if (data_manager->num_bilaterals == 0) {
        return;
    }
    NschurB = _DBT_ * _MINVDB_;
}

void ChSchurProductBilateral::operator()(const VectorType& x, VectorType& output) {
    output = NschurB * x;
}
