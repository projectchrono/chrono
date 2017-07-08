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

#ifndef CHITERATIVE_H
#define CHITERATIVE_H

#include <cmath>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMath.h"

namespace chrono {

// TO DO:
//   - add other Krylov methods (CG, MINRES, etc)
//   - enclose them in a class as in ChLinarAlgebra.h, with similar interfaces
//   - maybe move in core/ dir? or move both this and ChLinarAlgebra.h in a new dir 'source/numerical/' ?

/// TF-QMR
///
/// TRANSPOSE-FREE QUASI-MINIMAL-RESIDUAL solver for linear systems.
///
/// Iterative method to solve linear systems Ax=b, applies to
/// both symmetric and non symmetric matrices, does not
/// breakdown for singular matrices, accepts preconditioning.
///
///  x         = initial guess for X0 and also matrix to store final value;
///  b         = known term vector in Ax=b
///  SolveAX   = function which computes b"=Ax (gets x as 1st parameter,
///              outputs in b", the 2nd param, and may use some user data as 3rd param.)
///  M1_solve  = function which performs preconditioner M matrix solve as Out=M'*In
///  M2_solve  = function which performs preconditioner M matrix solve as Out=M'*In
///  min_kappa = threshold for convergence
///  max_iterations = limit on number of iterations
///  iter      = returns tot iterations done before exiting
///  error_code= returns error, if any:
///					0:   convergence within maximum iterations
///					1:   no convergence after maximum iterations
///					2:   breakdown in       tau
///					3:   breakdown in       alpha
///					4:   breakdown in       gamma
///					5:   breakdown in       rho
///  userdata  = generic pointer to whatever one needs (it will be passed to SolveAX
///              as 3rd parameter, so it may be useful to pass some pointer to application,
///              geometric or modeling data, etc., then avoiding the use of global vars.)

ChApi int ch_iterative_TFQMR(ChMatrix<>& x,
                             ChMatrix<>& b,
                             void (*SolveAX)(ChMatrix<>& inX, ChMatrix<>& outB, void* userdata),
                             void (*M1_solve)(ChMatrix<>& eIn, ChMatrix<>& eOut, void* userdata),
                             void (*M2_solve)(ChMatrix<>& eIn, ChMatrix<>& eOut, void* userdata),
                             double min_kappa,
                             int max_iterations,
                             int& iter,
                             int& error_code,
                             void* userdata);

/// As before, but with less parameters, easier to use.

ChApi int ch_iterative_TFQMR_easy(ChMatrix<>& A, ChMatrix<>& x, ChMatrix<>& b, double mkappa, int iterations);

}  // end namespace chrono

#endif
