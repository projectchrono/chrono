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

uint ChSolverMulticoreJacobi::Solve(ChSchurProduct& SchurProduct, ChProjectConstraints& Project, const uint max_iter, const uint size, const VectorType& r, VectorType& gamma) {
    if (size == 0) {
        return 0;
    }

    real& residual = data_manager->measures.solver.residual;
    real& objective_value = data_manager->measures.solver.objective_value;

    ml = gamma;
    ml_old = gamma;
    VectorType temp;
    temp.resize(size);
    VectorType deltal;
    deltal.resize(size);
    SparseMatrixType nschur_local;
    if (!data_manager->settings.solver.compute_N) {
        nschur_local = data_manager->host_data.D_T * data_manager->host_data.M_invD;
    }
    const SparseMatrixType& Nschur = data_manager->settings.solver.compute_N ? data_manager->host_data.Nschur : nschur_local;
    const VectorType& E = data_manager->host_data.E;
    VectorType D;
    ComputeInverseDiagonal(Nschur, D);
    // real eignenval = LargestEigenValue(SchurProduct, temp);

    for (current_iteration = 0; current_iteration < (signed)max_iter; current_iteration++) {
        real omega = .2;  // 2.0 / eignenval;//1.0 / 3.0;
        ml = ml_old - omega * D.cwiseProduct(Nschur * ml_old + E.cwiseProduct(ml_old) - r);

        Project(ml.data());
        gamma = ml;

        ml_old = ml;

        real gdiff = 1.0 / std::pow(data_manager->num_constraints, 2.0);
        SchurProduct(gamma, temp);

        objective_value = gamma.dot(0.5 * temp - r);

        temp = temp - r;
        temp = gamma - gdiff * (temp);
        Project(temp.data());
        temp = (1.0 / gdiff) * (gamma - temp);

        residual = temp.norm();

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
