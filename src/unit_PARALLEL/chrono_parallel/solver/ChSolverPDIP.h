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
// Authors: Daniel Melanz
// =============================================================================
//
// This file contains an implementation of PDIP.
// TODO: Replace the Dinv matrix with a function
// TODO: Get rid of as many matrices as possible (diaglambda could be replaced)
// TODO: Use better linear solve than cg
// =============================================================================

#ifndef CHSOLVERPDIP_H
#define CHSOLVERPDIP_H

#include "chrono_parallel/ChConfigParallel.h"
#include "chrono_parallel/solver/ChSolverParallel.h"

namespace chrono {
class CH_PARALLEL_API ChSolverPDIP: public ChSolverParallel {
public:

  ChSolverPDIP() :
    ChSolverParallel() {
  }
  ~ChSolverPDIP() {
  }

  void Solve() {
    if (num_constraints == 0) {
      return;
    }

    total_iteration += SolvePDIP(max_iteration, num_constraints,
        data_container->host_data.rhs_data,
        data_container->host_data.gamma_data);

    current_iteration = total_iteration;
  }

  // Solve using a more streamlined but harder to read version of the APGD method
  uint SolvePDIP(const uint max_iter,       // Maximum number of iterations
      const uint size,               // Number of unknowns
      custom_vector<real> &b,        // Rhs vector
      custom_vector<real> &x         // The vector of unknowns
  );

  // Compute the updated velocities
  // The solution for gamma is already here, so use it rather than having to copy it
  void ComputeImpulses();

  // Compute the residual for the solver
  real Res4(blaze::DynamicVector<real> & gamma,
      blaze::DynamicVector<real> & tmp);

  // Compute the Schur Complement Product, dst = N * src
  void SchurComplementProduct(blaze::DynamicVector<real> & src, blaze::DynamicVector<real> & dst);
  void initializeNewtonStepMatrix(blaze::DynamicVector<real> & gamma, blaze::DynamicVector<real> & lambda, blaze::DynamicVector<real> & f, const uint size);
  void initializeConstraintGradient(blaze::DynamicVector<real> & src, const uint size);
  void getConstraintVector(blaze::DynamicVector<real> & src, blaze::DynamicVector<real> & dst, const uint size);
  void updateConstraintGradient(blaze::DynamicVector<real> & src, const uint size);
  void updateNewtonStepMatrix(blaze::DynamicVector<real> & gamma, blaze::DynamicVector<real> & lambda, blaze::DynamicVector<real> & f, const uint size);
  void updateNewtonStepVector(blaze::DynamicVector<real> & gamma, blaze::DynamicVector<real> & lambda, blaze::DynamicVector<real> & f, real t, const uint size);
  void conjugateGradient(blaze::DynamicVector<real> & x);
  int preconditionedConjugateGradient(blaze::DynamicVector<real> & x, const uint size);
  void buildPreconditioner(const uint size);
  void applyPreconditioning(blaze::DynamicVector<real> & src, blaze::DynamicVector<real> & dst);
  void MultiplyByDiagMatrix(blaze::DynamicVector<real> & diagVec, blaze::DynamicVector<real> & src, blaze::DynamicVector<real> & dst);

  //PDIP specific vectors
  blaze::DynamicVector<real> gamma, f, r, lambda, r_d, r_g, ones, delta_gamma, delta_lambda, lambda_tmp, gamma_tmp;
  blaze::DynamicVector<real> r_cg, p_cg, z_cg, Ap_cg, prec_cg;
  blaze::CompressedMatrix<real> grad_f, M_hat, B, diaglambda, Dinv;

};
}
#endif

