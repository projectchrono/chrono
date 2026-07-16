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
    real& residual = data_manager->measures.solver.residual;
    real& objective_value = data_manager->measures.solver.objective_value;

    ml = gamma;
    ml_old = gamma;
    Project(ml.data());

    uint num_constraints = data_manager->num_constraints;
    uint num_contacts = data_manager->cd_data->num_rigid_contacts;
    uint num_rigid_particle_contacts = data_manager->cd_data->num_rigid_particle_contacts;
    uint num_bilaterals = data_manager->num_bilaterals;

    SparseMatrixType nschur_local;
    if (!data_manager->settings.solver.compute_N) {
        nschur_local = data_manager->host_data.D_T * data_manager->host_data.M_invD;
    }
    const SparseMatrixType& Nschur = data_manager->settings.solver.compute_N ? data_manager->host_data.Nschur : nschur_local;
    VectorType D;
    D.resize(num_constraints, false);

    auto diag = Nschur.diagonal();
    for (int index = 0; index < (signed)num_contacts; index++) {
        D[index] = diag[index] + diag[num_contacts + index * 2] + diag[num_contacts + index * 2 + 1];
        D[index] = 3.0 / D[index];
        D[num_contacts + index * 2 + 0] = D[index];
        D[num_contacts + index * 2 + 1] = D[index];
    }
    uint offset = data_manager->num_unilaterals;

    for (size_t i = 0; i < num_bilaterals; i++) {
        D[offset + i] = 1.0 / Nschur.coeff(offset + i, offset + i);
    }

    if (data_manager->num_particles) {
        offset += data_manager->num_bilaterals;

        for (size_t i = 0; i < data_manager->num_particles; i++) {
            D[offset + i] = 1.0 / Nschur.coeff(offset + i, offset + i);
        }

        offset += data_manager->num_particles;

        for (size_t i = 0; i < num_rigid_particle_contacts; i++) {
            if (data_manager->node_container->contact_mu == 0) {
                D[offset + i] = Nschur.coeff(offset + i, offset + i);
                D[offset + i] = 3.0 / D[offset + i];
            } else {
                D[offset + i] = diag[offset + i] + diag[offset + i * 2 + 0] + diag[offset + i * 2 + 1];
                D[offset + i] = 3.0 / D[offset + i];
                D[offset + i * 2 + 0] = D[offset + i];
                D[offset + i * 2 + 1] = D[offset + i];
            }
        }
    }
    int nc = num_contacts;
    int nfc = num_rigid_particle_contacts;
    for (current_iteration = 0; current_iteration < (signed)max_iter; current_iteration++) {
        real omega = .2;
        offset = 0;

        for (int i = 0; i < (signed)num_contacts; i++) {
            ml[offset + i] = ml[offset + i] - omega * D[offset + i] * (Nschur.row(offset + i * 1 + 0).dot(ml) - r[offset + i]);
            ml[nc + i * 2 + 0] = ml[nc + i * 2 + 0] - omega * D[nc + i * 2 + 0] * (Nschur.row(nc + i * 2 + 0).dot(ml) - r[nc + i * 2 + 0]);
            ml[nc + i * 2 + 1] = ml[nc + i * 2 + 1] - omega * D[nc + i * 2 + 1] * (Nschur.row(nc + i * 2 + 1).dot(ml) - r[nc + i * 2 + 1]);

            data_manager->rigid_rigid->Project_Single(i, ml.data());
        }

        offset += data_manager->num_unilaterals;

        for (size_t i = 0; i < num_bilaterals; i++) {
            ml[offset + i] = ml[offset + i] - omega * D[offset + i] * (Nschur.row(offset + i * 1 + 0).dot(ml) - r[offset + i]);
        }
        if (data_manager->num_particles) {
            offset += data_manager->num_bilaterals;

            for (size_t i = 0; i < data_manager->num_particles; i++) {
                ml[offset + i] = ml[offset + i] - omega * D[offset + i] * (Nschur.row(offset + i * 1 + 0).dot(ml) - r[offset + i]);
            }

            offset += data_manager->num_particles;

            for (size_t i = 0; i < num_rigid_particle_contacts; i++) {
                ml[offset + i] = ml[offset + i] - omega * D[offset + i] * (Nschur.row(offset + i * 1 + 0).dot(ml) - r[offset + i]);
                if (data_manager->node_container->contact_mu != 0) {
                    ml[offset + nfc + i * 2 + 0] =
                        ml[offset + nfc + i * 2 + 0] - omega * D[offset + nfc + i * 2 + 0] * (Nschur.row(offset + nfc + i * 2 + 0).dot(ml) - r[offset + nfc + i * 2 + 0]);
                    ml[offset + nfc + i * 2 + 1] =
                        ml[offset + nfc + i * 2 + 1] - omega * D[offset + nfc + i * 2 + 1] * (Nschur.row(offset + nfc + i * 2 + 1).dot(ml) - r[offset + nfc + i * 2 + 1]);
                }
                data_manager->node_container->Project(ml.data());
            }
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
