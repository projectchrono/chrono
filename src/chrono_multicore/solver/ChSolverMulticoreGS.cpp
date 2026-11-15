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

#include <cmath>

#include "chrono_multicore/solver/ChSolverMulticore.h"

using namespace chrono;

uint ChSolverMulticoreGS::Solve(ChSchurProduct& SchurProduct, ChProjectConstraints& Project, const uint max_iter, const uint size, const VectorType& r, VectorType& gamma) {
    if (size == 0) {
        return 0;
    }

    real& residual = data_manager->measures.solver.residual;
    real& objective_value = data_manager->measures.solver.objective_value;

    ml = gamma;
    ml_old = gamma;
    Project(ml.data());

    uint num_constraints = data_manager->num_constraints;
    uint nc = data_manager->cd_data->num_rigid_contacts;
    uint nfc = data_manager->cd_data->num_rigid_particle_contacts;
    uint num_bilaterals = data_manager->num_bilaterals;

    // Rows of rigid contacts updated in the current solver mode
    SolverMode mode = data_manager->settings.solver.local_solver_mode;
    bool sliding = (mode == SolverMode::SLIDING || mode == SolverMode::SPINNING);
    bool spinning = (mode == SolverMode::SPINNING);

    SparseMatrixType nschur_local;
    if (!data_manager->settings.solver.compute_N) {
        nschur_local = data_manager->host_data.D_T * data_manager->host_data.M_invD;
    }
    const SparseMatrixType& Nschur = data_manager->settings.solver.compute_N ? data_manager->host_data.Nschur : nschur_local;
    const VectorType& E = data_manager->host_data.E;
    VectorType D;
    ComputeInverseDiagonal(Nschur, D);

    real omega = .2;
    auto update_row = [&](uint i) { ml[i] = ml[i] - omega * D[i] * (Nschur.row(i).dot(ml) + E[i] * ml[i] - r[i]); };

    for (current_iteration = 0; current_iteration < (signed)max_iter; current_iteration++) {
        for (uint i = 0; i < nc; i++) {
            update_row(i);
            if (sliding) {
                update_row(nc + i * 2 + 0);
                update_row(nc + i * 2 + 1);
            }
            if (spinning) {
                update_row(3 * nc + i * 3 + 0);
                update_row(3 * nc + i * 3 + 1);
                update_row(3 * nc + i * 3 + 2);
            }
            data_manager->rigid_rigid->Project_Single(i, ml.data());
        }

        uint offset = data_manager->num_unilaterals;

        for (uint i = 0; i < num_bilaterals; i++) {
            update_row(offset + i);
        }
        if (data_manager->num_particles) {
            offset += num_bilaterals;

            for (uint i = 0; i < nfc; i++) {
                update_row(offset + i);
                if (data_manager->node_container->contact_mu != 0) {
                    update_row(offset + nfc + i * 2 + 0);
                    update_row(offset + nfc + i * 2 + 1);
                }
                data_manager->node_container->Project(ml.data());
            }

            offset += (data_manager->node_container->contact_mu == 0) ? nfc : 3 * nfc;

            for (uint i = offset; i < num_constraints; i++) {
                update_row(i);
            }
            data_manager->node_container->Project(ml.data());
        }

        gamma = ml;
        real gdiff = 1.0 / std::pow(data_manager->num_constraints, 2.0);
        SchurProduct(gamma, ml_old);

        objective_value = ml.dot(0.5 * ml_old - r);

        ml_old = ml_old - r;
        ml_old = gamma - gdiff * (ml_old);
        Project(ml_old.data());
        ml_old = (1.0 / gdiff) * (gamma - ml_old);

        residual = ml_old.norm();

        AtIterationEnd(residual, objective_value);

        if (data_manager->settings.solver.test_objective) {
            if (objective_value <= data_manager->settings.solver.tolerance_objective) {
                break;
            }
        } else {
            if (residual < data_manager->settings.solver.tol_speed) {
                break;
            }
        }
    }

    return current_iteration;
}
