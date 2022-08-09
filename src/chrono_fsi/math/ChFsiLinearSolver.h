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
// Author: Milad Rakhsha
// =============================================================================
//
// Class for solving a linear linear system via iterative methods.
// =============================================================================

#ifndef CHFSILINEARSOLVER_H_
#define CHFSILINEARSOLVER_H_

#include <ctype.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <typeinfo>
#include "cublas_v2.h"
#include "cusparse_v2.h"

#include "chrono_fsi/math/custom_math.h"
#include "chrono_fsi/ChDefinitionsFsi.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_solver
/// @{

/// Base class for solving linear systems on GPUs. 
/// Specific solution methods are implemented in derived classes.
class ChFsiLinearSolver {
  public:
    /// Constructor of the ChFsiLinearSolver class.
    ChFsiLinearSolver(SolverType msolver,
                      Real mrel_res = 1e-8,
                      Real mabs_res = 1e-4,
                      int mmax_iter = 1000,
                      bool mverbose = false) {
        rel_res = mrel_res;
        abs_res = mabs_res;
        max_iter = mmax_iter;
        verbose = mverbose;
        solver_type = msolver;
    };

    /// Destructor of the ChFsiLinearSolver class.
    virtual ~ChFsiLinearSolver() {}

    /// Return the solver type.
    SolverType GetType() { return solver_type; }

    /// Set verbose output from solver.
    void SetVerbose(bool mv) { verbose = mv; }

    /// Return whether or not verbose output is enabled.
    bool GetVerbose() const { return verbose; }

    /// Return the current residual.
    Real GetResidual() { return residual; }

    /// Return the number of current Iterations.
    int GetNumIterations() { return Iterations; }

    /// Set the maximum number of iterations.
    void SetIterationLimit(int numIter) { max_iter = numIter; }

    /// Set the maximum number of iterations.
    int GetIterationLimit() { return max_iter; }

    /// Set the absolute residual.
    void SetAbsRes(Real mabs_res) { abs_res = mabs_res; }

    /// Get the absolute residual.
    Real GetAbsRes() { return abs_res; }

    /// Get the relative residual.
    void SetRelRes(Real mrel_res) { rel_res = mrel_res; }

    /// Get the relative residual.
    Real GetRelRes() { return rel_res; }

    /// Return the solver status.
    /// - 0: unsuccessful
    /// - 1: successfully converged
    int GetSolverStatus() { return solver_status; }

    /// Solve linear system for x.
    virtual void Solve(int SIZE, int NNZ, Real* A, unsigned int* ArowIdx, unsigned int* AcolIdx, Real* x, Real* b) = 0;

  protected:
    Real rel_res = 1e-3;
    Real abs_res = 1e-6;
    int max_iter = 500;
    bool verbose = false;
    int Iterations = 0;
    Real residual = 1e5;
    int solver_status = 0;

  private:
    SolverType solver_type;
};

/// @} fsi_solver

}  // end namespace fsi
}  // end namespace chrono

#endif
