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

#ifndef CHFSILINEARSOLVER_BICGSTAB_H_
#define CHFSILINEARSOLVER_BICGSTAB_H_

#include <ctype.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <typeinfo>
#include "cublas_v2.h"
#include "cusparse_v2.h"

#include "chrono_fsi/sph/math/ChFsiLinearSolver.h"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsi_solver
/// @{

/// BiCG-Stab iterative linear solver.
class LinearSolverBiCGStab : public ChFsiLinearSolver {
  public:
    /// Constructor of the LinearSolverBiCGStab class.
    LinearSolverBiCGStab(Real mrel_res = 1e-8, Real mabs_res = 1e-4, int mmax_iter = 1000, bool mverbose = false)
        : ChFsiLinearSolver(SolverType::BICGSTAB, mrel_res, mabs_res, mmax_iter, mverbose) {}

    /// Destructor of the LinearSolverBiCGStab class.
    ~LinearSolverBiCGStab() {}

    /// Solves the linear system on the device.
    virtual void Solve(int SIZE, int NNZ, Real* A, unsigned int* ArowIdx, unsigned int* AcolIdx, Real* x, Real* b)
        override;

  private:
};

/// @} fsi_solver

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
