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

#include "chrono_multicore/solver/ChIterativeSolverMulticore.h"

using namespace chrono;

#define xstr(s) str(s)
#define str(s) #s

void ChIterativeSolverMulticoreNSC::RunTimeStep() {
    // Compute the offsets and number of constrains depending on the solver mode
    const auto num_rigid_contacts = data_manager->cd_data ? data_manager->cd_data->num_rigid_contacts : 0;

    if (data_manager->settings.solver.solver_mode == SolverMode::NORMAL) {
        data_manager->rigid_rigid->offset = 1;
        data_manager->num_unilaterals = 1 * num_rigid_contacts;
    } else if (data_manager->settings.solver.solver_mode == SolverMode::SLIDING) {
        data_manager->rigid_rigid->offset = 3;
        data_manager->num_unilaterals = 3 * num_rigid_contacts;
    } else if (data_manager->settings.solver.solver_mode == SolverMode::SPINNING) {
        data_manager->rigid_rigid->offset = 6;
        data_manager->num_unilaterals = 6 * num_rigid_contacts;
    }

    uint num_3dof_3dof = data_manager->node_container->GetNumConstraints();

    // Get the number of 3dof constraints, from the 3dof container in use right now

    // This is the total number of constraints
    data_manager->num_constraints = data_manager->num_unilaterals + data_manager->num_bilaterals + num_3dof_3dof;
    // Generate the mass matrix and compute M_inv_k
    ComputeInvMassMatrix();

    data_manager->host_data.gamma.resize(data_manager->num_constraints);
    data_manager->host_data.gamma.setZero();

    // Perform any setup tasks for all constraint types
    data_manager->rigid_rigid->Setup(data_manager);
    data_manager->bilateral->Setup(data_manager);
    data_manager->node_container->Setup3DOF(data_manager->num_unilaterals + data_manager->num_bilaterals);

    // Clear and reset solver history data and counters
    solver->current_iteration = 0;
    bilateral_solver->current_iteration = 0;
    data_manager->measures.solver.total_iteration = 0;
    data_manager->measures.solver.maxd_hist.clear();
    data_manager->measures.solver.maxdeltalambda_hist.clear();

    // Set pointers to constraint objects and perform setup actions for solver

    data_manager->system_timer.start("ChIterativeSolverMulticore_Setup");
    solver->Setup(data_manager);
    bilateral_solver->Setup(data_manager);
    data_manager->system_timer.stop("ChIterativeSolverMulticore_Setup");

    data_manager->system_timer.start("ChIterativeSolverMulticore_Matrices");
    ComputeD();
    ComputeE();
    ComputeR();
    ComputeN();
    data_manager->system_timer.stop("ChIterativeSolverMulticore_Matrices");

    data_manager->system_timer.start("ChIterativeSolverMulticore_Solve");

    data_manager->node_container->PreSolve();

    // // Recompute M_invk using the velocity that PreSolve() may have updated,
    // // then use it for both R_full and ComputeImpulses() to avoid a second
    // // M_inv * hf product later.
    // data_manager->host_data.M_invk.noalias() =
    //     data_manager->host_data.v + data_manager->host_data.M_inv * data_manager->host_data.hf;

    if (data_manager->num_constraints > 0) {
        data_manager->host_data.R_full.noalias() =
            // -data_manager->host_data.b - data_manager->host_data.D_T * data_manager->host_data.M_invk;
            -data_manager->host_data.b - data_manager->host_data.D_T *
                (data_manager->host_data.v + data_manager->host_data.M_inv * data_manager->host_data.hf);
    }
    SchurProductFull.Setup(data_manager);
    SchurProductBilateral.Setup(data_manager);
    ProjectFull.Setup(data_manager);

    PerformStabilization();

    if (data_manager->settings.solver.solver_mode == SolverMode::NORMAL || data_manager->settings.solver.solver_mode == SolverMode::SLIDING ||
        data_manager->settings.solver.solver_mode == SolverMode::SPINNING) {
        if (data_manager->settings.solver.max_iteration_normal > 0) {
            data_manager->settings.solver.local_solver_mode = SolverMode::NORMAL;
            SetR();
            data_manager->measures.solver.total_iteration += solver->Solve(SchurProductFull,
                                                                           ProjectFull,
                                                                           data_manager->settings.solver.max_iteration_normal,
                                                                           data_manager->num_constraints,
                                                                           data_manager->host_data.R,
                                                                           data_manager->host_data.gamma);
        }
    }
    if (data_manager->settings.solver.solver_mode == SolverMode::SLIDING || data_manager->settings.solver.solver_mode == SolverMode::SPINNING) {
        if (data_manager->settings.solver.max_iteration_sliding > 0) {
            data_manager->settings.solver.local_solver_mode = SolverMode::SLIDING;
            SetR();
            data_manager->measures.solver.total_iteration += solver->Solve(SchurProductFull,
                                                                           ProjectFull,
                                                                           data_manager->settings.solver.max_iteration_sliding,
                                                                           data_manager->num_constraints,
                                                                           data_manager->host_data.R,
                                                                           data_manager->host_data.gamma);
        }
    }
    if (data_manager->settings.solver.solver_mode == SolverMode::SPINNING) {
        if (data_manager->settings.solver.max_iteration_spinning > 0) {
            data_manager->settings.solver.local_solver_mode = SolverMode::SPINNING;
            SetR();
            data_manager->measures.solver.total_iteration += solver->Solve(SchurProductFull,
                                                                           ProjectFull,
                                                                           data_manager->settings.solver.max_iteration_spinning,
                                                                           data_manager->num_constraints,
                                                                           data_manager->host_data.R,
                                                                           data_manager->host_data.gamma);
        }
    }

    data_manager->Fc_current = false;
    data_manager->node_container->PostSolve();

    data_manager->system_timer.stop("ChIterativeSolverMulticore_Solve");

    ComputeImpulses();

    for (int i = 0; i < data_manager->measures.solver.maxd_hist.size(); i++) {
        AtIterationEnd(data_manager->measures.solver.maxd_hist[i], data_manager->measures.solver.maxdeltalambda_hist[i], i);
    }
    m_iterations = (int)data_manager->measures.solver.maxd_hist.size();
}

void ChIterativeSolverMulticoreNSC::ComputeD() {
    data_manager->system_timer.start("ChIterativeSolverMulticore_D");
    uint num_constraints = data_manager->num_constraints;
    if (num_constraints <= 0) {
        return;
    }

    uint num_dof = data_manager->num_dof;
    uint num_rigid_contacts = data_manager->cd_data ? data_manager->cd_data->num_rigid_contacts : 0;
    uint num_bilaterals = data_manager->num_bilaterals;
    uint num_unilaterals = data_manager->num_unilaterals;
    SolverMode solver_mode = data_manager->settings.solver.solver_mode;

    int num_normal = 1 * num_rigid_contacts;
    int num_tangential = 2 * num_rigid_contacts;
    int num_spinning = 3 * num_rigid_contacts;

    uint num_particle_particle = data_manager->node_container->GetNumConstraints();
    uint nnz_particle_particle = data_manager->node_container->GetNumNonZeros();

    SparseMatrixType& D_T = data_manager->host_data.D_T;
    SparseMatrixType& M_invD = data_manager->host_data.M_invD;
    const SparseMatrixType& M_inv = data_manager->host_data.M_inv;

    int num_rows = num_bilaterals + num_particle_particle;

    switch (solver_mode) {
        case SolverMode::NORMAL:
            num_rows += num_normal;
            break;
        case SolverMode::SLIDING:
            num_rows += num_normal + num_tangential;
            break;
        case SolverMode::SPINNING:
            num_rows += num_normal + num_tangential + num_spinning;
            break;
        default:
            break;
    }

    // Build per-row nnz counts for D_T
    D_T.resize(num_rows, num_dof);
    {
        Eigen::VectorXi d_nnz(num_rows);
        d_nnz.setZero();

        d_nnz.segment(0, num_normal).setConstant(12);
        if (solver_mode == SolverMode::SLIDING || solver_mode == SolverMode::SPINNING) {
            d_nnz.segment(num_normal, num_tangential).setConstant(12);
        }
        if (solver_mode == SolverMode::SPINNING) {
            d_nnz.segment(3 * (int)num_rigid_contacts, num_spinning).setConstant(6);
        }

        {
            int bil_off = (int)num_unilaterals;
            for (int index = 0; index < (signed)num_bilaterals; index++) {
                int cntr = data_manager->host_data.bilateral_mapping[index];
                switch (data_manager->host_data.bilateral_type[cntr]) {
                    case BilateralType::BODY_BODY:         d_nnz[bil_off + index] = 12; break;
                    case BilateralType::SHAFT_SHAFT:       d_nnz[bil_off + index] = 2;  break;
                    case BilateralType::SHAFT_BODY:        d_nnz[bil_off + index] = 7;  break;
                    case BilateralType::SHAFT_SHAFT_SHAFT: d_nnz[bil_off + index] = 3;  break;
                    case BilateralType::SHAFT_SHAFT_BODY:  d_nnz[bil_off + index] = 8;  break;
                    default: break;
                }
            }
        }

        if (num_particle_particle > 0) {
            int pp_off = (int)num_unilaterals + (int)num_bilaterals;
            int avg_nnz = ((int)nnz_particle_particle + (int)num_particle_particle - 1) /
                          (int)num_particle_particle;
            d_nnz.segment(pp_off, (int)num_particle_particle).setConstant(avg_nnz);
        }

        D_T.reserve(d_nnz);
    }

    M_invD.resize(0, 0);

    data_manager->rigid_rigid->GenerateSparsity();
    data_manager->bilateral->GenerateSparsity();
    data_manager->node_container->GenerateSparsity();

    // Move b code here so that it can be computed along side D
    VectorType& b = data_manager->host_data.b;
    b.resize(data_manager->num_constraints);
    b.setZero();

    data_manager->rigid_rigid->Build_D();
    data_manager->bilateral->Build_D();
    data_manager->node_container->Build_D();

    D_T.makeCompressed();

    // Using transpose() function will do in place transpose and copy
    data_manager->host_data.D = D_T.transpose();

    // For diagonal M_inv (use_full_inertia_tensor=false), avoid the general
    // sparse x sparse product by copying D then scaling each row by M_inv[i,i].
    // This is O(nnz_D) instead of Eigen's general spGEMM with its symbolic phase.
    if (!data_manager->settings.solver.use_full_inertia_tensor) {
        M_invD = data_manager->host_data.D;
        const int* M_inv_outer = M_inv.outerIndexPtr();
        const real* M_inv_vals = M_inv.valuePtr();
        const int* invD_outer = M_invD.outerIndexPtr();
        real* invD_vals = M_invD.valuePtr();
        const int n = (int)M_invD.outerSize();
        for (int i = 0; i < n; i++) {
            const real scale = (M_inv_outer[i] < M_inv_outer[i + 1])
                               ? M_inv_vals[M_inv_outer[i]] : real(0);
            for (int k = invD_outer[i]; k < invD_outer[i + 1]; k++)
                invD_vals[k] *= scale;
        }
    } else {
        M_invD = M_inv * data_manager->host_data.D;
    }
    data_manager->system_timer.stop("ChIterativeSolverMulticore_D");
}

void ChIterativeSolverMulticoreNSC::ComputeE() {
    data_manager->system_timer.start("ChIterativeSolverMulticore_E");
    if (data_manager->num_constraints <= 0) {
        return;
    }

    data_manager->host_data.E.resize(data_manager->num_constraints);
    data_manager->host_data.E.setZero();

    data_manager->rigid_rigid->Build_E();
    data_manager->bilateral->Build_E();
    data_manager->node_container->Build_E();

    data_manager->system_timer.stop("ChIterativeSolverMulticore_E");
}

void ChIterativeSolverMulticoreNSC::ComputeR() {
    data_manager->system_timer.start("ChIterativeSolverMulticore_R");
    if (data_manager->num_constraints <= 0) {
        return;
    }

    VectorType& R = data_manager->host_data.R_full;

    // B is now resized in the Jacobian function
    R.resize(data_manager->num_constraints);
    R.setZero();

    data_manager->rigid_rigid->Build_b();
    data_manager->bilateral->Build_b();
    data_manager->node_container->Build_b();

    // update right-hand side after pre-solve
    ////R = -b - D_T * M_invk;

    data_manager->system_timer.stop("ChIterativeSolverMulticore_R");
}

void ChIterativeSolverMulticoreNSC::ComputeN() {
    if (data_manager->settings.solver.compute_N == false) {
        return;
    }

    data_manager->system_timer.start("ChIterativeSolverMulticore_N");
    const SparseMatrixType& D_T = data_manager->host_data.D_T;
    SparseMatrixType& Nschur = data_manager->host_data.Nschur;
    Nschur = D_T * data_manager->host_data.M_invD;
    data_manager->system_timer.stop("ChIterativeSolverMulticore_N");
}

void ChIterativeSolverMulticoreNSC::SetR() {
    if (data_manager->num_constraints <= 0) {
        return;
    }

    VectorType& R = data_manager->host_data.R;
    const VectorType& R_full = data_manager->host_data.R_full;

    uint num_rigid_contacts = 0;
    uint num_rigid_particle = 0;
    if (data_manager->cd_data) {
        num_rigid_contacts = data_manager->cd_data->num_rigid_contacts;
        num_rigid_particle = data_manager->cd_data->num_rigid_particle_contacts * 3;
    }
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;
    uint num_particles = data_manager->num_particles;
    R.resize(data_manager->num_constraints);
    R.setZero();

    if (data_manager->settings.solver.local_solver_mode == data_manager->settings.solver.solver_mode) {
        R = R_full;
    } else {
        R.segment(num_unilaterals, num_bilaterals) = R_full.segment(num_unilaterals, num_bilaterals);
        R.segment(num_unilaterals + num_bilaterals, num_rigid_particle) = R_full.segment(num_unilaterals + num_bilaterals, num_rigid_particle);

        // TODO: Set R in the associated 3dof container
        R.segment(num_unilaterals + num_bilaterals + num_rigid_particle, num_particles) =
            R_full.segment(num_unilaterals + num_bilaterals + num_rigid_particle, num_particles);

        switch (data_manager->settings.solver.local_solver_mode) {
            case SolverMode::BILATERAL: {
            } break;

            case SolverMode::NORMAL: {
                R.segment(0, num_rigid_contacts) = R_full.segment(0, num_rigid_contacts);
            } break;

            case SolverMode::SLIDING: {
                R.segment(0, num_rigid_contacts) = R_full.segment(0, num_rigid_contacts);
                R.segment(num_rigid_contacts, num_rigid_contacts * 2) = R_full.segment(num_rigid_contacts, num_rigid_contacts * 2);
            } break;

            case SolverMode::SPINNING: {
                R.segment(0, num_rigid_contacts) = R_full.segment(0, num_rigid_contacts);
                R.segment(num_rigid_contacts, num_rigid_contacts * 2) = R_full.segment(num_rigid_contacts, num_rigid_contacts * 2);
                R.segment(num_rigid_contacts * 3, num_rigid_contacts * 3) = R_full.segment(num_rigid_contacts * 3, num_rigid_contacts * 3);
            } break;
        }
    }
}

void ChIterativeSolverMulticoreNSC::ComputeImpulses() {
    // M_invk = v + M_inv*hf was refreshed in RunTimeStep() after PreSolve().
    const VectorType& M_invk = data_manager->host_data.M_invk;
    const VectorType& gamma = data_manager->host_data.gamma;
    VectorType& v = data_manager->host_data.v;

    if (data_manager->num_constraints > 0) {
        v.noalias() = M_invk + data_manager->host_data.M_invD * gamma;
    } else {
        v = M_invk;
    }
}

void ChIterativeSolverMulticoreNSC::PreSolve() {
    // Currently not supported, might be added back in the future
}

void ChIterativeSolverMulticoreNSC::ChangeSolverType(SolverType type) {
    data_manager->settings.solver.solver_type = type;

    if (this->solver) {
        delete (this->solver);
    }
    switch (type) {
        case SolverType::APGD:
            solver = new ChSolverMulticoreAPGD();
            break;
        case SolverType::APGDREF:
            solver = new ChSolverMulticoreAPGDREF();
            break;
        case SolverType::BB:
            solver = new ChSolverMulticoreBB();
            break;
        case SolverType::SPGQP:
            solver = new ChSolverMulticoreSPGQP();
            break;
        case SolverType::JACOBI:
            solver = new ChSolverMulticoreJacobi();
            break;
        case SolverType::GAUSS_SEIDEL:
            solver = new ChSolverMulticoreGS();
            break;
        default:
            break;
    }
}
