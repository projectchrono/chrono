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

#pragma once

#include "chrono_parallel/solver/ChSolverParallel.h"

namespace chrono {

class CH_PARALLEL_API ChSolverParallelPDIP : public ChSolverParallel {
 public:
  ChSolverParallelPDIP() : ChSolverParallel() {}
  ~ChSolverParallelPDIP() {}

  void Solve() {
    if (data_manager->num_constraints == 0) {
      return;
    }
    data_manager->system_timer.start("ChSolverParallel_Solve");
    const CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;
    uint num_dof = data_manager->num_dof;
    uint num_contacts = data_manager->num_rigid_contacts;
    uint num_bilaterals = data_manager->num_bilaterals;
    uint num_constraints = data_manager->num_constraints;
    uint num_unilaterals = data_manager->num_unilaterals;
    uint nnz_bilaterals = data_manager->nnz_bilaterals;
    uint nnz_unilaterals = 6 * 6 * data_manager->num_rigid_contacts;

    int nnz_total = nnz_unilaterals + nnz_bilaterals;

    D_T.reserve(nnz_total);
    D_T.resize(num_constraints, num_dof, false);

    D.reserve(nnz_total);
    D.resize(num_dof, num_constraints, false);

    M_invD.reserve(nnz_total);
    M_invD.resize(num_dof, num_constraints, false);

    SubMatrixType D_n_T = blaze::submatrix(D_T, 0, 0, num_contacts, num_dof);
    SubMatrixType D_t_T =
        blaze::submatrix(D_T, num_contacts, 0, 2 * num_contacts, num_dof);
    SubMatrixType D_b_T =
        blaze::submatrix(D_T, num_unilaterals, 0, num_bilaterals, num_dof);

    D_n_T = data_manager->host_data.D_n_T;
    D_t_T = data_manager->host_data.D_t_T;
    D_b_T = data_manager->host_data.D_b_T;

    SubMatrixType D_n = blaze::submatrix(D, 0, 0, num_dof, num_contacts);
    SubMatrixType D_t =
        blaze::submatrix(D, 0, num_contacts, num_dof, 2 * num_contacts);
    SubMatrixType D_b =
        blaze::submatrix(D, 0, num_unilaterals, num_dof, num_bilaterals);

    D_n = data_manager->host_data.D_n;
    D_t = data_manager->host_data.D_t;
    D_b = data_manager->host_data.D_b;

    SubMatrixType M_invD_n = blaze::submatrix(M_invD, 0, 0, num_dof, num_contacts);
    SubMatrixType M_invD_t =
        blaze::submatrix(M_invD, 0, num_contacts, num_dof, 2 * num_contacts);
    SubMatrixType M_invD_b =
        blaze::submatrix(M_invD, 0, num_unilaterals, num_dof, num_bilaterals);

    M_invD_n = data_manager->host_data.M_invD_n;
    M_invD_t = data_manager->host_data.M_invD_t;
    M_invD_b = data_manager->host_data.M_invD_b;

    data_manager->measures.solver.total_iteration += SolvePDIP(
        max_iteration, data_manager->num_constraints, data_manager->host_data.R, data_manager->host_data.gamma);
    data_manager->system_timer.stop("ChSolverParallel_Solve");
  }

  // Solve using the primal-dual interior point method
  uint SolvePDIP(const uint max_iter,                  // Maximum number of iterations
                 const uint size,                      // Number of unknowns
                 const DynamicVector<real>& b,  // Rhs vector
                 DynamicVector<real>& x         // The vector of unknowns
                 );

  // Compute the residual for the solver
  real Res4(DynamicVector<real>& gamma, DynamicVector<real>& tmp);

  // Compute the Schur Complement Product, dst = N * src
  void SchurComplementProduct(DynamicVector<real>& src, DynamicVector<real>& dst);
  void initializeNewtonStepMatrix(DynamicVector<real>& gamma,
                                  DynamicVector<real>& lambda,
                                  DynamicVector<real>& f,
                                  const uint size);
  void initializeConstraintGradient(DynamicVector<real>& src, const uint size);
  void getConstraintVector(DynamicVector<real>& src, DynamicVector<real>& dst, const uint size);
  void updateConstraintGradient(DynamicVector<real>& src, const uint size);
  void updateNewtonStepMatrix(DynamicVector<real>& gamma,
                              DynamicVector<real>& lambda,
                              DynamicVector<real>& f,
                              const uint size);
  void updateNewtonStepVector(DynamicVector<real>& gamma,
                              DynamicVector<real>& lambda,
                              DynamicVector<real>& f,
                              real t,
                              const uint size);
  void conjugateGradient(DynamicVector<real>& x);
  int preconditionedConjugateGradient(DynamicVector<real>& x, const uint size);
  void buildPreconditioner(const uint size);
  void applyPreconditioning(DynamicVector<real>& src, DynamicVector<real>& dst);
  void MultiplyByDiagMatrix(DynamicVector<real>& diagVec,
                            DynamicVector<real>& src,
                            DynamicVector<real>& dst);

  // PDIP specific vectors
  DynamicVector<real> gamma, f, r, lambda, r_d, r_g, ones, delta_gamma, delta_lambda, lambda_tmp, gamma_tmp;
  DynamicVector<real> r_cg, p_cg, z_cg, Ap_cg, prec_cg;
  CompressedMatrix<real> grad_f, M_hat, B, diaglambda, Dinv;

  CompressedMatrix<real> D_T, D, M_invD;
};
}

