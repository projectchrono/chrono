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

uint ChSolverMulticoreJacobi::Solve(ChSchurProduct& SchurProduct,
                                    ChProjectConstraints& Project,
                                    const uint max_iter,
                                    const uint size,
                                    const DynamicVector<real>& r,
                                    DynamicVector<real>& gamma) {
    if (size == 0) {
        return 0;
    }

    real& residual = data_manager->measures.solver.residual;
    real& objective_value = data_manager->measures.solver.objective_value;

    uint num_constraints = data_manager->num_constraints;
    uint num_contacts = data_manager->cd_data->num_rigid_contacts;
    uint num_rigid_particle_contacts = data_manager->cd_data->num_rigid_particle_contacts;
    uint num_bilaterals = data_manager->num_bilaterals;
    ml = gamma;
    ml_old = gamma;
    DynamicVector<real> temp;
    temp.resize(size);
    DynamicVector<real> deltal;
    deltal.resize(size);
    CompressedMatrix<real> Nschur = data_manager->host_data.D_T * data_manager->host_data.M_invD;
    DynamicVector<real> D;
    D.resize(num_constraints, false);
    // real eignenval = LargestEigenValue(SchurProduct, temp);

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

    if (data_manager->num_particles) {
        offset += data_manager->num_bilaterals;

        for (size_t i = 0; i < data_manager->num_particles; i++) {
            D[offset + i] = 1.0 / Nschur(offset + i, offset + i);
        }

        offset += data_manager->num_particles;

        for (size_t i = 0; i < num_rigid_particle_contacts; i++) {
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

    for (current_iteration = 0; current_iteration < (signed)max_iter; current_iteration++) {
        real omega = .2;  // 2.0 / eignenval;//1.0 / 3.0;
        ml = ml_old - omega * D * (Nschur * ml_old - r);

        Project(ml.data());
        gamma = ml;

        ml_old = ml;

        real gdiff = 1.0 / std::pow(data_manager->num_constraints, 2.0);
        SchurProduct(gamma, temp);

        objective_value = (gamma, (0.5 * temp - r));

        temp = temp - r;
        temp = gamma - gdiff * (temp);
        Project(temp.data());
        temp = (1.0 / gdiff) * (gamma - temp);

        residual = Sqrt((double)(temp, temp));

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