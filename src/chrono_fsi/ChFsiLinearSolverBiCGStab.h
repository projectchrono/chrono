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

#ifndef CHFSILINEARSOLVER_BICGSTAB_H_
#define CHFSILINEARSOLVER_BICGSTAB_H_

#include <chrono_fsi/ChFsiLinearSolver.h>
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

/// @brief GPU implementation of BICGSTAB method via CUDA libraries
class ChFsiLinearSolverBiCGStab : public ChFsiLinearSolver {
  public:
    ChFsiLinearSolverBiCGStab(double mrel_res = 1e-8,  ///< relative residual of the linear solver
                              double mabs_res = 1e-4,  ///< absolute residual of the linear solver
                              int mmax_iter = 1000,    ///< Maximum number of iteration of the linear solver
                              bool mverbose = false    ///< Verbosity of solver during each solve stage
                              )
        : ChFsiLinearSolver(mrel_res, mabs_res, mmax_iter, mverbose) {}

    virtual ~ChFsiLinearSolverBiCGStab() {}

    /// returns the type of the linear solver
    virtual SolverType GetType() override { return SolverType::BICGSTAB; }

    /// implements the abstract method of the base class using BICGSTAB
    virtual void Solve(int SIZE,               ///< size of the matrix in Ax=b
                       int NNZ,                ///< number of nonzeros in A matrix
                       double* A,              ///< pointer to the matrix A stored in CSR format
                       unsigned int* ArowIdx,  ///< accumulation of NNZ of each row of matrix A
                       unsigned int* AcolIdx,  ///< column index of each element of A and hence is of length NNZ
                       double* x,              ///< pointer to the solution vector x
                       double* b               ///< pointer to the solution right hand side vector b in Ax=b
                       ) override;

  private:
};

/// @} fsi_solver
}  // end namespace fsi
}  // end namespace chrono
#endif /* CHFSILINEARSOLVER_H_ */
