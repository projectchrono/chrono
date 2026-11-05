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
//
// This file contains the base class used for all multicore iterative solvers.
// All of the functions are defined here, with the implementation of each solver
// in it's specific cpp file.
//
// =============================================================================

#include "chrono_multicore/solver/ChSolverMulticore.h"

using namespace chrono;

void ChProjectConstraints::operator()(real* data) {
    data_manager->system_timer.start("ChSolverMulticore_Project");
    data_manager->rigid_rigid->Project(data);
    data_manager->node_container->Project(data);
    data_manager->system_timer.stop("ChSolverMulticore_Project");
}

ChSolverMulticore::ChSolverMulticore() {
    current_iteration = 0;
    rigid_rigid = NULL;
    three_dof = NULL;
    fem = NULL;
    bilateral = NULL;
}

//=================================================================================================================================

void ChSolverMulticore::ComputeSRhs(custom_vector<real>& gamma,
                                    const custom_vector<real>& rhs,
                                    custom_vector<real3>& vel_data,
                                    custom_vector<real3>& omg_data,
                                    custom_vector<real>& b) {
    // TODO change SHRS to use blaze
    // ComputeImpulses(gamma, vel_data, omg_data);
    // rigid_rigid->ComputeS(rhs, vel_data, omg_data, b);
}

bool init_eigen_vec = 0;

real ChSolverMulticore::LargestEigenValue(ChSchurProduct& SchurProduct, VectorType& temp, real lambda) {
    eigen_vec.resize(temp.size());
    if (init_eigen_vec == 0) {
        eigen_vec.setOnes();
        init_eigen_vec = 1;
    }

    if (lambda != 0) {
        SchurProduct(eigen_vec, temp);
        eigen_vec = 1.0 / lambda * temp;
    }
    real lambda_old = 0;

    for (int i = 0; i < data_manager->settings.solver.max_power_iteration; i++) {
        SchurProduct(eigen_vec, temp);
        lambda = temp.norm();
        if (lambda == 0) {
            return 1;
        }
        printf("Lambda: %.20f \n", lambda);
        if (std::abs(lambda_old - lambda) < data_manager->settings.solver.power_iter_tolerance) {
            break;
        }
        eigen_vec = 1.0 / lambda * temp;
        lambda_old = lambda;
    }
    return lambda;
}

void ChSolverMulticore::ComputeInverseDiagonal(const SparseMatrixType& Nschur, VectorType& D) {
    const VectorType& E = data_manager->host_data.E;
    const SolverMode mode = data_manager->settings.solver.local_solver_mode;
    const bool sliding = (mode == SolverMode::SLIDING || mode == SolverMode::SPINNING);
    const bool spinning = (mode == SolverMode::SPINNING);

    const uint num_constraints = data_manager->num_constraints;
    const uint nc = data_manager->cd_data->num_rigid_contacts;
    const uint nfc = data_manager->cd_data->num_rigid_particle_contacts;

    D.setZero(num_constraints);

    auto diag = Nschur.diagonal();
    auto N_ii = [&](uint i) { return diag[i] + E[i]; };

    // Rigid contacts: normal rows, then two tangential rows per contact (SLIDING, SPINNING), then one spinning and two
    // rolling rows per contact (SPINNING)
    for (uint i = 0; i < nc; i++) {
        if (!sliding) {
            D[i] = 1.0 / N_ii(i);
            continue;
        }
        uint t = nc + i * 2;
        D[i] = 3.0 / (N_ii(i) + N_ii(t) + N_ii(t + 1));
        D[t] = D[i];
        D[t + 1] = D[i];
        if (spinning) {
            uint s = 3 * nc + i * 3;
            D[s] = 3.0 / (N_ii(s) + N_ii(s + 1) + N_ii(s + 2));
            D[s + 1] = D[s];
            D[s + 2] = D[s];
        }
    }

    uint offset = data_manager->num_unilaterals;
    for (uint i = 0; i < data_manager->num_bilaterals; i++) {
        D[offset + i] = 1.0 / N_ii(offset + i);
    }
    offset += data_manager->num_bilaterals;

    if (data_manager->num_particles) {
        // Rigid-particle contacts: normal rows, followed by two tangential rows per contact if there is friction
        const bool friction = (data_manager->node_container->contact_mu != 0);
        for (uint i = 0; i < nfc; i++) {
            if (!friction) {
                D[offset + i] = 3.0 / N_ii(offset + i);
                continue;
            }
            uint t = offset + nfc + i * 2;
            D[offset + i] = 3.0 / (N_ii(offset + i) + N_ii(t) + N_ii(t + 1));
            D[t] = D[offset + i];
            D[t + 1] = D[offset + i];
        }
        offset += friction ? 3 * nfc : nfc;

        // Remaining 3-DOF constraints (fluid density and viscosity, particle-particle contacts)
        for (uint i = offset; i < num_constraints; i++) {
            D[i] = 1.0 / N_ii(i);
        }
    }
}
