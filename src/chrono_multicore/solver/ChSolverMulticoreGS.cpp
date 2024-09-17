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

#if BLAZE_MAJOR_VERSION == 2
    #include <blaze/math/SparseRow.h>
#endif

#include <blaze/math/CompressedMatrix.h>

using namespace chrono;

uint ChSolverMulticoreGS::Solve(ChSchurProduct& SchurProduct,
                                ChProjectConstraints& Project,
                                const uint max_iter,
                                const uint size,
                                const DynamicVector<real>& r,
                                DynamicVector<real>& gamma) {
    real& residual = data_manager->measures.solver.residual;
    real& objective_value = data_manager->measures.solver.objective_value;

    ml = gamma;
    ml_old = gamma;
    Project(ml.data());

    uint num_constraints = data_manager->num_constraints;
    uint num_contacts = data_manager->cd_data->num_rigid_contacts;
    uint num_rigid_fluid_contacts = data_manager->cd_data->num_rigid_fluid_contacts;
    uint num_bilaterals = data_manager->num_bilaterals;

    CompressedMatrix<real> Nschur = data_manager->host_data.D_T * data_manager->host_data.M_invD;
    DynamicVector<real> D;
    D.resize(num_constraints, false);

    for (int index = 0; index < (signed)num_contacts; index++) {
        D[index] = Nschur(index, index) + Nschur(num_contacts + index * 2 + 0, num_contacts + index * 2 + 0) +
                   Nschur(num_contacts + index * 2 + 1, num_contacts + index * 2 + 1);
        D[index] = 3.0 / D[index];
        D[num_contacts + index * 2 + 0] = D[index];
        D[num_contacts + index * 2 + 1] = D[index];
    }
    uint offset = data_manager->num_unilaterals;

    for (size_t i = 0; i < num_bilaterals; i++) {
        D[offset + i] = 1.0 / Nschur(offset + i, offset + i);
    }

    if (data_manager->num_fluid_bodies) {
        offset += data_manager->num_bilaterals;

        for (size_t i = 0; i < data_manager->num_fluid_bodies; i++) {
            D[offset + i] = 1.0 / Nschur(offset + i, offset + i);
        }

        offset += data_manager->num_fluid_bodies;

        for (size_t i = 0; i < num_rigid_fluid_contacts; i++) {
            if (data_manager->node_container->contact_mu == 0) {
                D[offset + i] = Nschur(offset + i, offset + i);
                D[offset + i] = 3.0 / D[offset + i];
            } else {
                D[offset + i] = Nschur(offset + i, offset + i) + Nschur(offset + i * 2 + 0, offset + i * 2 + 0) +
                                Nschur(offset + i * 2 + 1, offset + i * 2 + 1);
                D[offset + i] = 3.0 / D[offset + i];
                D[offset + i * 2 + 0] = D[offset + i];
                D[offset + i * 2 + 1] = D[offset + i];
            }
        }
    }
    int nc = num_contacts;
    int nfc = num_rigid_fluid_contacts;
    for (current_iteration = 0; current_iteration < (signed)max_iter; current_iteration++) {
        real omega = .2;
        offset = 0;

        for (int i = 0; i < (signed)num_contacts; i++) {
            ml[offset + i] = ml[offset + i] - omega * D[offset + i] *
                                                  ((row(Nschur, offset + i * 1 + 0),
                                                    blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) -
                                                   r[offset + i]);
            ml[nc + i * 2 + 0] =
                ml[nc + i * 2 + 0] -
                omega * D[nc + i * 2 + 0] *
                    ((row(Nschur, nc + i * 2 + 0), blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) -
                     r[nc + i * 2 + 0]);
            ml[nc + i * 2 + 1] =
                ml[nc + i * 2 + 1] -
                omega * D[nc + i * 2 + 1] *
                    ((row(Nschur, nc + i * 2 + 1), blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) -
                     r[nc + i * 2 + 1]);

            data_manager->rigid_rigid->Project_Single(i, ml.data());
        }

        offset += data_manager->num_unilaterals;

        for (size_t i = 0; i < num_bilaterals; i++) {
            ml[offset + i] = ml[offset + i] - omega * D[offset + i] *
                                                  ((row(Nschur, offset + i * 1 + 0),
                                                    blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) -
                                                   r[offset + i]);
        }
        if (data_manager->num_fluid_bodies) {
            offset += data_manager->num_bilaterals;

            for (size_t i = 0; i < data_manager->num_fluid_bodies; i++) {
                ml[offset + i] = ml[offset + i] - omega * D[offset + i] *
                                                      ((row(Nschur, offset + i * 1 + 0),
                                                        blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) -
                                                       r[offset + i]);
            }

            offset += data_manager->num_fluid_bodies;

            for (size_t i = 0; i < num_rigid_fluid_contacts; i++) {
                ml[offset + i] = ml[offset + i] - omega * D[offset + i] *
                                                      ((row(Nschur, offset + i * 1 + 0),
                                                        blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) -
                                                       r[offset + i]);
                if (data_manager->node_container->contact_mu != 0) {
                    ml[offset + nfc + i * 2 + 0] = ml[offset + nfc + i * 2 + 0] -
                                                   omega * D[offset + nfc + i * 2 + 0] *
                                                       ((row(Nschur, offset + nfc + i * 2 + 0),
                                                         blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) -
                                                        r[offset + nfc + i * 2 + 0]);
                    ml[offset + nfc + i * 2 + 1] = ml[offset + nfc + i * 2 + 1] -
                                                   omega * D[offset + nfc + i * 2 + 1] *
                                                       ((row(Nschur, offset + nfc + i * 2 + 1),
                                                         blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) -
                                                        r[offset + nfc + i * 2 + 1]);
                }
                data_manager->node_container->Project(ml.data());
            }
        }

        gamma = ml;
        real gdiff = 1.0 / std::pow(data_manager->num_constraints, 2.0);
        SchurProduct(gamma, ml_old);

        objective_value = (ml, (0.5 * ml_old - r));

        ml_old = ml_old - r;
        ml_old = gamma - gdiff * (ml_old);
        Project(ml_old.data());
        ml_old = (1.0 / gdiff) * (gamma - ml_old);

        residual = Sqrt((double)(ml_old, ml_old));

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
