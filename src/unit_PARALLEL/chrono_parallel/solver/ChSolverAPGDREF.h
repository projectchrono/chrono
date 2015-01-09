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
// Authors: Hammad Mazhar, Daniel Melanz
// =============================================================================
//
// This file contains an implementation of APGD that is exactly like the thesis
// =============================================================================

#ifndef CHSOLVERAPGD_H
#define CHSOLVERAPGD_H

#include "chrono_parallel/ChConfigParallel.h"
#include "chrono_parallel/solver/ChSolverParallel.h"

namespace chrono {
class CH_PARALLEL_API ChSolverAPGDREF: public ChSolverParallel {
public:

	ChSolverAPGDREF() :
      ChSolverParallel() {
  }
  ~ChSolverAPGDREF() {
  }

  void Solve() {
    if (num_constraints == 0) {
      return;
    }

    total_iteration += SolveAPGDREF(max_iteration, num_constraints,
        data_container->host_data.R,
        data_container->host_data.gamma);

    current_iteration = total_iteration;
  }

  // Solve using a more streamlined but harder to read version of the APGD method
  uint SolveAPGDREF(const uint max_iter,       // Maximum number of iterations
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
