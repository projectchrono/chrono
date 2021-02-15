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

#ifndef CHFSILINEARSOLVER_GMRES_H_
#define CHFSILINEARSOLVER_GMRES_H_

#include <ctype.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <typeinfo>
#include "cublas_v2.h"
#include "cusparse_v2.h"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"
#include "chrono_fsi/math/ChFsiLinearSolver.h"

namespace chrono {
namespace fsi {

typedef char MM_typecode[4];
/// @addtogroup fsi_math
/// @{
class ChFsiLinearSolverGMRES : public ChFsiLinearSolver {
  public:
    ChFsiLinearSolverGMRES(double mrel_res = 1e-8, double mabs_res = 1e-4, int mmax_iter = 1000, bool mverbose = false)
        : ChFsiLinearSolver(mrel_res, mabs_res, mmax_iter, mverbose, SolverType::GMRES) {}

    virtual ~ChFsiLinearSolverGMRES() {}

    /// Returns the solver type
    virtual SolverType GetType() override { return SolverType::GMRES; }

    /// Solves the linear system on the device
    virtual void Solve(int SIZE, int NNZ, double* A, unsigned int* ArowIdx, unsigned int* AcolIdx, double* x, double* b)
        override;

    /// Sets the restart parameter in the GMRES method
    void SetRestart(int R) { restart = R; }

  private:
    int restart = 100;
};
/// @} fsi_math

}  // end namespace fsi
}  // end namespace chrono
#endif
