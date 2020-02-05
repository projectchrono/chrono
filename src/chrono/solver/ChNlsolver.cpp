// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include <cmath>

#include "chrono/solver/ChNlsolver.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMathematics.h"

namespace chrono {

void ChNonlinearSolver::JacobianCompute(void (*m_func)(ChVectorRef mx, ChVectorRef res, void* my_data),
                                        ChVectorRef mx,
                                        ChVectorRef res,
                                        void* my_data,
                                        ChMatrixRef mJ,
                                        double diff_step) {
    if (diff_step <= 0)
        diff_step = BDF_STEP_LOW;

    ChVectorDynamic<> dres = res;
    ChVectorDynamic<> dx = mx;

    for (int i = 0; i < mx.size(); i++) {
        dx(i) = mx(i) + diff_step;
        (*m_func)(dx, dres, my_data);
        for (int j = 0; j < res.size(); j++) {
            mJ(j, i) = (dres(j) - res(j)) / diff_step;
        }
        dx(i) = mx(i);
    }
}

double ChNonlinearSolver::NewtonRaphson(void (*m_func)(ChVectorRef mx, ChVectorRef res, void* my_data),
                                        void (*m_jacob)(ChVectorRef mx, ChMatrixRef mJ, void* my_data),
                                        ChVectorRef mx,
                                        void* my_data,
                                        int maxiters,
                                        double tolerance) {
    ChVectorDynamic<> res;
    ChVectorDynamic<> delta;
    ChMatrixDynamic<> jac;

    auto vars = mx.size();
    res.setZero(vars);
    jac.setZero(vars, vars);

    int iters = 0;
    double residual = 0;

    while (true) {
        if (iters >= maxiters)
            break;

        // compute residuals
        (*m_func)(mx, res, my_data);
        residual = res.lpNorm<Eigen::Infinity>();
        if (residual <= tolerance)
            break;

        // compute jacobian
        if (m_jacob)
            (*m_jacob)(mx, jac, my_data);
        else {
            ChNonlinearSolver::JacobianCompute(m_func, mx, res, my_data, jac, BDF_STEP_LOW);
        }

        // solve jac * delta = res
        delta = jac.colPivHouseholderQr().solve(res);
        mx -= delta;

        iters++;
    }
    return residual;
}

}  // end namespace chrono
