//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include <cmath>

#include "chrono/physics/ChNlsolver.h"
#include "chrono/core/ChLinearAlgebra.h"

namespace chrono {

void ChNonlinearSolver::JacobianCompute(void (*m_func)(ChMatrix<>* mx, ChMatrix<>* res, void* my_data),
                                        ChMatrix<>* mx,
                                        ChMatrix<>* res,
                                        void* my_data,
                                        ChMatrix<>* mJ,
                                        double diff_step) {
    if (diff_step <= 0)
        diff_step = BDF_STEP_LOW;
    int jrows = res->GetRows();
    int jcols = mx->GetRows();

    ChMatrixDynamic<> dres;
    ChMatrixDynamic<> dx;

    dres.Reset(jrows, 1);
    dx.Reset(jcols, 1);

    dres.CopyFromMatrix(*res);
    dx.CopyFromMatrix(*mx);

    for (int i = 0; i < jcols; i++) {
        dx.SetElement(i, 0, mx->GetElement(i, 0) + diff_step);
        (*m_func)(&dx, &dres, my_data);
        for (int j = 0; j < jrows; j++) {
            mJ->SetElement(j, i, (dres.GetElement(j, 0) - res->GetElement(j, 0)) / diff_step);
        }
        dx.SetElement(i, 0, mx->GetElement(i, 0));
    }
}

double ChNonlinearSolver::NewtonRaphson(void (*m_func)(ChMatrix<>* mx, ChMatrix<>* res, void* my_data),
                                        void (*m_jacob)(ChMatrix<>* mx, ChMatrix<>* mJ, void* my_data),
                                        ChMatrix<>* mx,
                                        void* my_data,
                                        int maxiters,
                                        double tolerance) {
    ChMatrixDynamic<> res;
    ChMatrixDynamic<> jac;
    ChMatrixDynamic<> delta;

    double residual = 0;
    int vars = mx->GetRows();

    res.Reset(vars, 1);
    delta.Reset(vars, 1);
    jac.Reset(vars, vars);

    int iters = 0;

    while (true) {
        if (iters >= maxiters)
            break;

        // compute residuals

        (*m_func)(mx, &res, my_data);
        residual = res.NormInf();
        if (residual <= tolerance)
            break;

        // compute jacobian
        if (m_jacob)
            (*m_jacob)(mx, &jac, my_data);
        else {
            ChNonlinearSolver::JacobianCompute(m_func, mx, &res, my_data, &jac, BDF_STEP_LOW);
        }
        // solve LU for delta
        ChLinearAlgebra::Solve_LinSys(jac, &res, &delta);
        delta.MatrNeg();
        mx->MatrInc(delta);

        iters++;
    }
    return residual;
}

}  // end namespace chrono
