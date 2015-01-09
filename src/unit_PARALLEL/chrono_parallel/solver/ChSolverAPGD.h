// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// This file contains an implementation of APGD that is more optimized.
// =============================================================================

#ifndef CHSOLVERAPGD_H
#define CHSOLVERAPGD_H

#include "chrono_parallel/ChConfigParallel.h"
#include "chrono_parallel/solver/ChSolverParallel.h"

namespace chrono {
class CH_PARALLEL_API ChSolverAPGD: public ChSolverParallel {
public:

  ChSolverAPGD() :
      ChSolverParallel() {
  }
  ~ChSolverAPGD() {
  }

  void Solve() {
    if (num_constraints == 0) {
      return;
    }

    total_iteration += SolveAPGDBlaze(max_iteration, num_constraints,
        data_container->host_data.R,
        data_container->host_data.gamma);

    current_iteration = total_iteration;
  }

  // Solve using a more streamlined but harder to read version of the APGD method
  uint SolveAPGDBlaze(const uint max_iter,       // Maximum number of iterations
      const uint size,               // Number of unknowns
      const blaze::DynamicVector<real>& b,    // Rhs vector
      blaze::DynamicVector<real>& x     // The vector of unknowns
      );

  // Compute the residual for the solver
  real Res4(blaze::DynamicVector<real> & gamma,
      blaze::DynamicVector<real> & tmp);

  // Compute the Schur Complement Product, dst = N * src
  void SchurComplementProduct(blaze::DynamicVector<real> & src,
      blaze::DynamicVector<real> & dst);

  //APGD specific vectors
  blaze::DynamicVector<real> gamma_hat;
  blaze::DynamicVector<real> gammaNew, g, y, gamma, yNew, r, tmp;

};
}
#endif
