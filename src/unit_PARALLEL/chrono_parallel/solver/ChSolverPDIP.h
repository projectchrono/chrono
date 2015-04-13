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

#include "chrono_parallel/solver/ChSolverParallel.h"

namespace chrono {

class CH_PARALLEL_API ChSolverPDIP : public ChSolverParallel {
 public:
  ChSolverPDIP() : ChSolverParallel() {}
  ~ChSolverPDIP() {}

  void Solve() {
    if (data_container->num_constraints == 0) {
      return;
    }
    data_container->system_timer.start("ChSolverParallel_Solve");
    const CompressedMatrix<real>& M_inv = data_container->host_data.M_inv;
    uint num_dof = data_container->num_dof;
    uint num_contacts = data_container->num_rigid_contacts;
    uint num_bilaterals = data_container->num_bilaterals;
    uint num_constraints = data_container->num_constraints;
    uint num_unilaterals = data_container->num_unilaterals;
    uint nnz_bilaterals = data_container->nnz_bilaterals;
    uint nnz_unilaterals = 6 * 6 * data_container->num_rigid_contacts;

    int nnz_total = nnz_unilaterals + nnz_bilaterals;

    D_T.reserve(nnz_total);
    D_T.resize(num_constraints, num_dof, false);

    D.reserve(nnz_total);
    D.resize(num_dof, num_constraints, false);

    M_invD.reserve(nnz_total);
    M_invD.resize(num_dof, num_constraints, false);

    blaze::SparseSubmatrix<CompressedMatrix<real> > D_n_T = blaze::submatrix(D_T, 0, 0, num_contacts, num_dof);
    blaze::SparseSubmatrix<CompressedMatrix<real> > D_t_T =
        blaze::submatrix(D_T, num_contacts, 0, 2 * num_contacts, num_dof);
    blaze::SparseSubmatrix<CompressedMatrix<real> > D_b_T =
        blaze::submatrix(D_T, num_unilaterals, 0, num_bilaterals, num_dof);

    D_n_T = data_container->host_data.D_n_T;
    D_t_T = data_container->host_data.D_t_T;
    D_b_T = data_container->host_data.D_b_T;

    blaze::SparseSubmatrix<CompressedMatrix<real> > D_n = blaze::submatrix(D, 0, 0, num_dof, num_contacts);
    blaze::SparseSubmatrix<CompressedMatrix<real> > D_t =
        blaze::submatrix(D, 0, num_contacts, num_dof, 2 * num_contacts);
    blaze::SparseSubmatrix<CompressedMatrix<real> > D_b =
        blaze::submatrix(D, 0, num_unilaterals, num_dof, num_bilaterals);

    D_n = data_container->host_data.D_n;
    D_t = data_container->host_data.D_t;
    D_b = data_container->host_data.D_b;

    blaze::SparseSubmatrix<CompressedMatrix<real> > M_invD_n = blaze::submatrix(M_invD, 0, 0, num_dof, num_contacts);
    blaze::SparseSubmatrix<CompressedMatrix<real> > M_invD_t =
        blaze::submatrix(M_invD, 0, num_contacts, num_dof, 2 * num_contacts);
    blaze::SparseSubmatrix<CompressedMatrix<real> > M_invD_b =
        blaze::submatrix(M_invD, 0, num_unilaterals, num_dof, num_bilaterals);

    M_invD_n = data_container->host_data.M_invD_n;
    M_invD_t = data_container->host_data.M_invD_t;
    M_invD_b = data_container->host_data.M_invD_b;

    data_container->measures.solver.total_iteration += SolvePDIP(
        max_iteration, data_container->num_constraints, data_container->host_data.R, data_container->host_data.gamma);
    data_container->system_timer.stop("ChSolverParallel_Solve");
  }

  // Solve using the primal-dual interior point method
  uint SolvePDIP(const uint max_iter,                  // Maximum number of iterations
                 const uint size,                      // Number of unknowns
                 const blaze::DynamicVector<real>& b,  // Rhs vector
                 blaze::DynamicVector<real>& x         // The vector of unknowns
                 );

  // Compute the residual for the solver
  real Res4(blaze::DynamicVector<real>& gamma, blaze::DynamicVector<real>& tmp);

  // Compute the Schur Complement Product, dst = N * src
  void SchurComplementProduct(blaze::DynamicVector<real>& src, blaze::DynamicVector<real>& dst);
  void initializeNewtonStepMatrix(blaze::DynamicVector<real>& gamma,
                                  blaze::DynamicVector<real>& lambda,
                                  blaze::DynamicVector<real>& f,
                                  const uint size);
  void initializeConstraintGradient(blaze::DynamicVector<real>& src, const uint size);
  void getConstraintVector(blaze::DynamicVector<real>& src, blaze::DynamicVector<real>& dst, const uint size);
  void updateConstraintGradient(blaze::DynamicVector<real>& src, const uint size);
  void updateNewtonStepMatrix(blaze::DynamicVector<real>& gamma,
                              blaze::DynamicVector<real>& lambda,
                              blaze::DynamicVector<real>& f,
                              const uint size);
  void updateNewtonStepVector(blaze::DynamicVector<real>& gamma,
                              blaze::DynamicVector<real>& lambda,
                              blaze::DynamicVector<real>& f,
                              real t,
                              const uint size);
  void conjugateGradient(blaze::DynamicVector<real>& x);
  int preconditionedConjugateGradient(blaze::DynamicVector<real>& x, const uint size);
  void buildPreconditioner(const uint size);
  void applyPreconditioning(blaze::DynamicVector<real>& src, blaze::DynamicVector<real>& dst);
  void MultiplyByDiagMatrix(blaze::DynamicVector<real>& diagVec,
                            blaze::DynamicVector<real>& src,
                            blaze::DynamicVector<real>& dst);

  // PDIP specific vectors
  blaze::DynamicVector<real> gamma, f, r, lambda, r_d, r_g, ones, delta_gamma, delta_lambda, lambda_tmp, gamma_tmp;
  blaze::DynamicVector<real> r_cg, p_cg, z_cg, Ap_cg, prec_cg;
  blaze::CompressedMatrix<real> grad_f, M_hat, B, diaglambda, Dinv;

  blaze::CompressedMatrix<real> D_T, D, M_invD;
};
}

#endif
