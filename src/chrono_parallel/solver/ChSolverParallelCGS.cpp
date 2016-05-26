#include "chrono_parallel/solver/ChSolverParallelCGS.h"

using namespace chrono;

uint ChSolverParallelCGS::SolveCGS(const uint max_iter,
                                   const uint size,
                                   DynamicVector<real>& mb,
                                   DynamicVector<real>& ml) {
  real& residual = data_manager->measures.solver.residual;
  real& objective_value = data_manager->measures.solver.objective_value;

  r.resize(size);
  qhat.resize(size);
  vhat.resize(size);
  uhat.resize(size);

  ShurProduct(ml, r);  // r = data_manager->host_data.D_T *
                       // (data_manager->host_data.M_invD * ml);
  r = mb - r;
  p = r;
  q = r;

  u = r;

  real normb = Sqrt((mb, mb));
  rtilde = r;

  if (normb == 0.0) {
    normb = 1;
  }

  if ((Sqrt((r, r)) / normb) <= data_manager->settings.solver.tolerance) {
    return 0;
  }

  for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
    rho_1 = (rtilde, r);

    if (rho_1 == 0) {
      break;
    }

    if (current_iteration > 0) {
      beta = rho_1 / rho_2;

      u = r + beta * q;
      p = u + beta * (q + beta * p);
    }

    phat = p;
    ShurProduct(phat, vhat);  // vhat = data_manager->host_data.D_T *
                              // (data_manager->host_data.M_invD * phat);
    alpha = rho_1 / (rtilde, vhat);
    q = u - alpha * vhat;
    uhat = (u + q);
    ml = ml + alpha * uhat;
    ShurProduct(uhat, qhat);  // qhat = data_manager->host_data.D_T *
                              // (data_manager->host_data.M_invD * uhat);
    r = r - alpha * qhat;
    rho_2 = rho_1;
    residual = (Sqrt((r, r)) / normb);

    objective_value = GetObjective(ml, mb);
    AtIterationEnd(residual, objective_value);

    if (residual < data_manager->settings.solver.tolerance) {
      break;
    }
  }
  Project(ml.data());

  return current_iteration;
}
