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

#ifndef CHNLSOLVER_H
#define CHNLSOLVER_H

#include <cmath>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix.h"

namespace chrono {

///
/// Solver for systems of nonlinear equations.
///
///  The solver is provided as a static function, so it can be called
/// without the need of creating a class instance. That is, you just need to
/// type: ChNonlinearSolver::NewtonRaphson(etc...)
///
///

class ChApi ChNonlinearSolver {
  public:
    /// Function which computes the jacobian matrix of a nonlinear vectorial
    /// function (i.e. set of N scalar functions) respect to M variables.
    /// It is used internally by NewtonRaphson, but it can also be used by the user.
    /// If you want to use JacobianCompute, remember that matrices mJ, mx and res
    /// must be already allocated with the correct sizes (i.e. mJ(N,M), mx(M,1), res(N,1) )
    /// Also, res must be already computed, for current mx, and mx must be the
    /// current state in which the jacobian is desired.
    /// \test

    static void JacobianCompute(void (*m_func)(ChMatrix<>* mx, ChMatrix<>* res, void* my_data),
                                ChMatrix<>* mx,
                                ChMatrix<>* res,
                                void* my_data,
                                ChMatrix<>* mJ,
                                double diff_step);

    /// Static function which solves numerically a set of N nonlinear equations in N unknowns.
    /// You must pass the address of the function m_func() which computes the
    /// residuals values of the nonlinear equations (being zero when satisfied).
    /// Your m_func must return the N residuals in the column matrix 'res', given
    /// the values of the variables in column 'mx', each time it is called -no need
    /// to create res and mx inside your m_func!!!  Also, the generic pointer my_data
    /// is passed to my_func just for your convenience, for auxiliary data.
    /// You can pass the m_jacob() function address if you have a _fast_ custom routine to
    /// compute the jacobian mJ at mx, otherwise set it to NULL and a generic numerical
    /// method will be used by default.
    /// \return  Return value is norm of residual vector (should be near zero after root finding).
    /// \test

    static double NewtonRaphson(void (*m_func)(ChMatrix<>* mx, ChMatrix<>* res, void* my_data),
                                void (*m_jacob)(ChMatrix<>* mx, ChMatrix<>* mJ, void* my_data),
                                ChMatrix<>* mx,
                                void* my_data,
                                int maxiters,
                                double tolerance);
};

}  // end namespace chrono

#endif
