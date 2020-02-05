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
// Class for solving a linear linear system via iterative methods.//
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

namespace chrono {
namespace fsi {

typedef char MM_typecode[4];

/// @addtogroup fsi_solver
/// @{

/// @brief Base class of the FSI GPU-based linear solvers.
/// This is an abstract class and specific solvers must implement the solve procedure
class ChFsiLinearSolver {
  public:
    enum class SolverType { BICGSTAB = 0, GMRES, CR, CG, SAP };

    /// Constructor of the ChFsiLinearSolver
    ChFsiLinearSolver(double mrel_res = 1e-8,  ///< relative residual of the linear solver
                      double mabs_res = 1e-4,  ///< absolute residual of the linear solver
                      int mmax_iter = 1000,    ///< Maximum number of iteration of the linear solver
                      bool mverbose = false    ///< Verbosity of solver during each solve stage
    ) {
        rel_res = mrel_res;
        abs_res = mabs_res;
        max_iter = mmax_iter;
        verbose = mverbose;
    };

    virtual ~ChFsiLinearSolver(){};

    /// Return type of the solver. Derived classes should override this function.
    virtual SolverType GetType() = 0;

    /// Set verbose output from solver.
    void SetVerbose(bool mv) { verbose = mv; }
    /// Return whether or not verbose output is enabled.
    bool GetVerbose() const { return verbose; }

    /// Return the current residual
    double GetResidual() { return residual; }
    /// Return the number of current Iterations
    int GetNumIterations() { return Iterations; }

    /// Set and Get for the max_iter
    void SetIterationLimit(int numIter) { max_iter = numIter; }
    int GetIterationLimit() { return max_iter; }

    /// Set and Get for the abs_res and
    void SetAbsRes(double mabs_res) { abs_res = mabs_res; }
    double GetAbsRes() { return abs_res; }
    void SetRelRes(double mrel_res) { rel_res = mrel_res; }
    double GetRelRes() { return rel_res; }

    /// Return the solver status
    /// 0: unsuccessful
    /// 1: successfully converged
    int GetSolverStatus() { return solver_status; }

    /// The child class should override this function and solve for x
    virtual void Solve(int SIZE,               ///< size of the matrix in Ax=b
                       int NNZ,                ///< number of nonzeros in A matrix
                       double* A,              ///< pointer to the matrix A stored in CSR format
                       unsigned int* ArowIdx,  ///< accumulation of NNZ of each row of matrix A
                       unsigned int* AcolIdx,  ///< column index of each element of A and hence is of length NNZ
                       double* x,              ///< pointer to the solution vector x
                       double* b               ///< pointer to the solution right hand side vector b in Ax=b
                       ) = 0;

  protected:
    double rel_res = 1e-3;
    double abs_res = 1e-6;
    int max_iter = 500;
    bool verbose = false;
    int Iterations = 0;
    double residual = 1e5;
    int solver_status = 0;

  private:
    SolverType solver;
};
/// @} fsi_solver

}  // end namespace fsi
}  // end namespace chrono
#endif /* CHFSILINEARSOLVER_H_ */
