#include "chrono_parallel/solver/ChSolverParallelPDIP.h"
#include <blaze/math/CompressedVector.h>

#include "core/ChFileutils.h"
#include "core/ChStream.h"

#include <algorithm>

using namespace chrono;

unsigned int offset = 3;

#define _index_ i* offset

real ChSolverParallelPDIP::Res4(DynamicVector<real>& gamma, DynamicVector<real>& tmp) {
  real gdiff = 1e-6;
  SchurComplementProduct(gamma, tmp);
  tmp = tmp + r;
  tmp = gamma - gdiff * (tmp);
  Project(tmp.data());
  tmp = (1.0 / gdiff) * (gamma - tmp);

  return Sqrt((real)(tmp, tmp));
}

void ChSolverParallelPDIP::SchurComplementProduct(DynamicVector<real>& src, DynamicVector<real>& dst) {
  dst = D_T * (M_invD * src);
}

void ChSolverParallelPDIP::getConstraintVector(DynamicVector<real>& src,
                                               DynamicVector<real>& dst,
                                               const uint size) {
#pragma omp parallel for
  for (int i = 0; i < data_manager->num_rigid_contacts; i++) {
    dst[i] = 0.5 * (pow(src[_index_ + 1], 2) + pow(src[_index_ + 2], 2) -
                    pow(data_manager->host_data.fric_rigid_rigid[i].x, 2) * pow(src[_index_], 2));
    dst[i + data_manager->num_rigid_contacts] = -src[_index_];
  }
}

void ChSolverParallelPDIP::initializeConstraintGradient(DynamicVector<real>& src, const uint size) {
  //#pragma omp parallel for
  for (int i = 0; i < data_manager->num_rigid_contacts; i++) {
    grad_f.append(i, _index_ + 0, -pow(data_manager->host_data.fric_rigid_rigid[i].x, 2) * src[_index_ + 0]);
    grad_f.append(i, _index_ + 1, src[_index_ + 1]);
    grad_f.append(i, _index_ + 2, src[_index_ + 2]);
    grad_f.finalize(i);
  }
  for (int i = 0; i < data_manager->num_rigid_contacts; i++) {
    grad_f.append(i + data_manager->num_rigid_contacts, _index_, -1);
    grad_f.finalize(i + data_manager->num_rigid_contacts);
  }
}

void ChSolverParallelPDIP::updateConstraintGradient(DynamicVector<real>& src, const uint size) {
#pragma omp parallel for
  for (int i = 0; i < data_manager->num_rigid_contacts; i++) {
    grad_f(i, _index_ + 0) = -pow(data_manager->host_data.fric_rigid_rigid[i].x, 2) * src[_index_ + 0];
    grad_f(i, _index_ + 1) = src[_index_ + 1];
    grad_f(i, _index_ + 2) = src[_index_ + 2];
  }
#pragma omp parallel for
  for (int i = 0; i < data_manager->num_rigid_contacts; i++) {
    grad_f(i + data_manager->num_rigid_contacts, _index_) = -1;
  }
}

void ChSolverParallelPDIP::initializeNewtonStepMatrix(DynamicVector<real>& gamma,
                                                      DynamicVector<real>& lambda,
                                                      DynamicVector<real>& f,
                                                      const uint size) {
  for (int i = 0; i < data_manager->num_rigid_contacts; i++) {
    M_hat.append(_index_ + 0, _index_ + 0, -pow(data_manager->host_data.fric_rigid_rigid[i].x, 2) * lambda[i]);
    M_hat.finalize(_index_ + 0);
    M_hat.append(_index_ + 1, _index_ + 1, lambda[i]);
    M_hat.finalize(_index_ + 1);
    M_hat.append(_index_ + 2, _index_ + 2, lambda[i]);
    M_hat.finalize(_index_ + 2);
  }
  updateConstraintGradient(gamma, size);
  B = trans(grad_f);

  for (int i = 0; i < 2 * data_manager->num_rigid_contacts; i++) {
    diaglambda.append(i, i, lambda[i]);
    diaglambda.finalize(i);

    Dinv.append(i, i, -1 / f[i]);
    Dinv.finalize(i);
  }
}

void ChSolverParallelPDIP::updateNewtonStepMatrix(DynamicVector<real>& gamma,
                                                  DynamicVector<real>& lambda,
                                                  DynamicVector<real>& f,
                                                  const uint size) {
#pragma omp parallel for
  for (int i = 0; i < data_manager->num_rigid_contacts; i++) {
    M_hat(_index_ + 0, _index_ + 0) = -pow(data_manager->host_data.fric_rigid_rigid[i].x, 2) * lambda[i];
    M_hat(_index_ + 1, _index_ + 1) = lambda[i];
    M_hat(_index_ + 2, _index_ + 2) = lambda[i];
  }
  updateConstraintGradient(gamma, size);
  B = trans(grad_f);

#pragma omp parallel for
  for (int i = 0; i < 2 * data_manager->num_rigid_contacts; i++) {
    diaglambda(i, i) = lambda[i];

    Dinv(i, i) = -1 / f[i];
  }
}

void ChSolverParallelPDIP::MultiplyByDiagMatrix(DynamicVector<real>& diagVec,
                                                DynamicVector<real>& src,
                                                DynamicVector<real>& dst) {
#pragma omp parallel for
  for (int i = 0; i < 2 * data_manager->num_rigid_contacts; i++) {
    dst[i] = diagVec[i] * src[i];
  }
}

void ChSolverParallelPDIP::updateNewtonStepVector(DynamicVector<real>& gamma,
                                                  DynamicVector<real>& lambda,
                                                  DynamicVector<real>& f,
                                                  real t,
                                                  const uint size) {
  updateConstraintGradient(gamma, size);
  r_d = D_T * (M_invD * gamma) + r + trans(grad_f) * lambda;
  MultiplyByDiagMatrix(lambda, f, r_g);
  r_g = -(1 / t) * ones - r_g;
}

void ChSolverParallelPDIP::conjugateGradient(DynamicVector<real>& x) {
  real rsold_cg = 0;
  real rsnew_cg = 0;
  real alpha_cg = 0;
  r_cg = (B * (Dinv * r_g) - r_d) - (D_T * M_invD + M_hat + B * Dinv * diaglambda * grad_f) * x;
  p_cg = r_cg;
  rsold_cg = (r_cg, r_cg);

  for (int i = 0; i < 10 * gamma.size(); i++) {
    Ap_cg = (D_T * M_invD + M_hat + B * Dinv * diaglambda * grad_f) * p_cg;
    alpha_cg = rsold_cg / (p_cg, Ap_cg);
    x = x + alpha_cg * p_cg;
    r_cg = r_cg - alpha_cg * Ap_cg;
    rsnew_cg = (r_cg, r_cg);
    if (Sqrt(rsnew_cg) < data_manager->settings.solver.tol_speed / 100.0) {
      return;
    }
    p_cg = r_cg + rsnew_cg / rsold_cg * p_cg;
    rsold_cg = rsnew_cg;
  }
}

void ChSolverParallelPDIP::buildPreconditioner(const uint size) {
  prec_cg.resize(size);
  CompressedMatrix<real> A = D_T * M_invD + M_hat + B * Dinv * diaglambda * grad_f;
#pragma omp parallel for
  for (int i = 0; i < prec_cg.size(); i++) {
    prec_cg[i] = A(i, i);
  }
}

void ChSolverParallelPDIP::applyPreconditioning(DynamicVector<real>& src, DynamicVector<real>& dst) {
#pragma omp parallel for
  for (int i = 0; i < prec_cg.size(); i++) {
    dst[i] = src[i] / prec_cg[i];
  }
}

int ChSolverParallelPDIP::preconditionedConjugateGradient(DynamicVector<real>& x, const uint size) {
  buildPreconditioner(size);
  int iter = 0;
  real rsold_cg = 0;
  real rsnew_cg = 0;
  real alpha_cg = 0;
  r_cg = (B * (Dinv * r_g) - r_d) - (D_T * M_invD + M_hat + B * Dinv * diaglambda * grad_f) * x;
  applyPreconditioning(r_cg, z_cg);
  p_cg = z_cg;
  rsold_cg = (r_cg, z_cg);

  for (int i = 0; i < 1000 * gamma.size(); i++) {
    iter++;
    Ap_cg = (D_T * M_invD + M_hat + B * Dinv * diaglambda * grad_f) * p_cg;
    alpha_cg = rsold_cg / (p_cg, Ap_cg);
    x = x + alpha_cg * p_cg;
    r_cg = r_cg - alpha_cg * Ap_cg;
    applyPreconditioning(r_cg, z_cg);
    rsnew_cg = (z_cg, r_cg);
    if (Sqrt(rsnew_cg) < data_manager->settings.solver.tol_speed / 100.0) {
      return iter;
    }
    p_cg = z_cg + rsnew_cg / rsold_cg * p_cg;
    rsold_cg = rsnew_cg;
  }

  return iter;
}

uint ChSolverParallelPDIP::SolvePDIP(const uint max_iter,
                                     const uint size,
                                     const DynamicVector<real>& b,
                                     DynamicVector<real>& x) {
  bool verbose = false;
  if (verbose)
    std::cout << "Number of constraints: " << size << "\nNumber of variables  : " << data_manager->num_rigid_bodies
              << std::endl;

  real& residual = data_manager->measures.solver.residual;
  real& objective_value = data_manager->measures.solver.objective_value;


  data_manager->system_timer.start("ChSolverParallel_solverA");
  int totalKrylovIterations = 0;

  uint num_contacts = data_manager->num_rigid_contacts;

  // Initialize scalars
  real mu = 10;
  real alpha = 0.001;  // should be [0.01, 0.1]
  real beta = 0.8;     // should be [0.3, 0.8]
  real eta_hat = 0;
  real obj = 0;
  real newobj = 0;
  real t = 0;
  real s = 1;
  real s_max = 1;
  real norm_rt = 0;

  // Initialize vectors
  gamma.resize(size);
  gamma_tmp.resize(size);
  r.resize(size);
  f.resize(2 * num_contacts);
  lambda.resize(2 * num_contacts);
  lambda_tmp.resize(2 * num_contacts);
  ones.resize(2 * num_contacts);
  r_d.resize(size);
  r_g.resize(2 * num_contacts);
  delta_gamma.resize(size);
  delta_lambda.resize(size);

  r_cg.resize(size);
  p_cg.resize(size);
  z_cg.resize(size);
  Ap_cg.resize(size);

  // Initialize matrices
  grad_f.reset();
  grad_f.resize(2 * num_contacts, size);
  grad_f.reserve(4 * num_contacts);  // there are (4*num_contacts) nonzero entries

  M_hat.reset();
  M_hat.resize(size, size);
  M_hat.reserve(size);  // there are (size) nonzero entries

  B.reset();
  B.resize(size, 2 * num_contacts);
  B.reserve(4 * num_contacts);  // there are (4*num_contacts) nonzero entries

  diaglambda.reset();
  diaglambda.resize(2 * num_contacts, 2 * num_contacts);
  diaglambda.reserve(2 * num_contacts);  // there are (2 * num_contacts) nonzero entries

  Dinv.reset();
  Dinv.resize(2 * num_contacts, 2 * num_contacts);
  Dinv.reserve(2 * num_contacts);  // there are (2 * num_contacts) nonzero entries

#pragma omp parallel for
  for (int i = 0; i < size; i++) {
    delta_gamma[i] = 0.0;
    gamma[i] = 0.0;
    r[i] = -b[i];  // convert r to a blaze vector
  }

#pragma omp parallel for
  for (int i = 0; i < num_contacts; i++) {
    gamma[3 * i] = 1.0;  // provide an initial guess!
  }

  // (1) f = f(gamma_0)
  getConstraintVector(gamma, f, size);

// (2) lambda_0 = -1/f
#pragma omp parallel for
  for (int i = 0; i < f.size(); i++) {
    ones[i] = 1.0;
    lambda[i] = -1 / f[i];
  }
  initializeConstraintGradient(gamma, size);
  initializeNewtonStepMatrix(gamma, lambda, f, size);

  // (3) for k := 0 to N_max
  for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
    // (4) f = f(gamma_k)
    getConstraintVector(gamma, f, size);

    // (5) eta_hat = -f^T * lambda_k
    eta_hat = -(f, lambda);

    // (6) t = mu*m/eta_hat
    t = mu * (f.size()) / eta_hat;

    // (7) A = A(gamma_k, lambda_k, f)
    updateConstraintGradient(gamma, size);
    updateNewtonStepMatrix(gamma, lambda, f, size);

    // (8) r_t = r_t(gamma_k, lambda_k, t)
    updateNewtonStepVector(gamma, lambda, f, t, size);

    // (9) Solve the linear system A * y = -r_t
    int krylovIter = preconditionedConjugateGradient(delta_gamma, size);
    totalKrylovIterations += krylovIter;
    delta_lambda = Dinv * (diaglambda * (grad_f * delta_gamma) - r_g);

// (10) s_max = sup{s in [0,1]|lambda+s*delta_lambda>=0} = min{1,min{-lambda_i/delta_lambda_i|delta_lambda_i < 0 }}
#pragma omp parallel for
    for (int i = 0; i < lambda.size(); i++) {
      lambda_tmp[i] = -lambda[i] / delta_lambda[i];
      if (delta_lambda[i] >= 0)
        lambda_tmp[i] = 1.0;
    }

    s_max = Min(real(1.0), (real)blaze::min(lambda_tmp));

    // (11) s = 0.99 * s_max
    s = 0.99 * s_max;

    // (12) while Max(f(gamma_k + s * delta_gamma) > 0)
    gamma_tmp = gamma + s * delta_gamma;
    getConstraintVector(gamma_tmp, lambda_tmp, size);
    while (blaze::max(lambda_tmp) > 0) {
      // (13) s = beta * s
      s = beta * s;
      gamma_tmp = gamma + s * delta_gamma;
      getConstraintVector(gamma_tmp, lambda_tmp, size);

      // (14) endwhile
    }

    // (15) while norm(r_t(gamma_k + s * delta_gamma, lambda_k + s * delta_lambda),2) > (1-alpha*s)*norm(r_t,2)
    norm_rt = Sqrt((r_d, r_d) + (r_g, r_g));
    lambda_tmp = lambda + s * delta_lambda;
    getConstraintVector(gamma_tmp, f, size);
    updateNewtonStepVector(gamma_tmp, lambda_tmp, f, t, size);
    while (Sqrt((r_d, r_d) + (r_g, r_g)) > (1 - alpha * s) * norm_rt) {
      // (16) s = beta * s
      s = beta * s;
      gamma_tmp = gamma + s * delta_gamma;
      lambda_tmp = lambda + s * delta_lambda;
      getConstraintVector(gamma_tmp, f, size);
      updateNewtonStepVector(gamma_tmp, lambda_tmp, f, t, size);

      // (17) endwhile
    }

    // (18) gamma_(k+1) = gamma_k + s * delta_gamma
    gamma = gamma + s * delta_gamma;

    // (19) lambda_(k+1) = lamda_k + s * delta_lambda
    lambda = lambda + s * delta_lambda;

    // (20) r = r(gamma_(k+1))
    residual = Sqrt((r_g, r_g));  // Res4(gamma, gamma_tmp);

    // (21) if r < tau
    AtIterationEnd(residual, objective_value);
    if (verbose)
      std::cout << "Residual: " << residual << ", Iter: " << current_iteration
                << ", Krylov Iter: " << totalKrylovIterations << std::endl;

    if (residual < data_manager->settings.solver.tol_speed) {
      // (22) break
      break;

      // (23) endif
    }

    // (24) endfor
  }

// (25) return Value at time step t_(l+1), gamma_(l+1) := gamma_(k+1)
#pragma omp parallel for
  for (int i = 0; i < size; i++) {
    x[i] = gamma[i];
  }

  data_manager->system_timer.stop("ChSolverParallel_solverA");

  return current_iteration;
}
