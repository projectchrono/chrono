#include "chrono_parallel/solver/ChSolverCGS.h"

using namespace chrono;

uint ChSolverCGS::SolveCGS(const uint max_iter, const uint size, blaze::DynamicVector<real>& mb, blaze::DynamicVector<real>& ml) {
  real& residual = data_container->measures.solver.residual;
  real& objective_value = data_container->measures.solver.objective_value;
  custom_vector<real>& iter_hist = data_container->measures.solver.iter_hist;


  r.resize(size);
  qhat.resize(size);
  vhat.resize(size);
  uhat.resize(size);

  ShurProduct(ml, r);    // r = data_container->host_data.D_T *
                         // (data_container->host_data.M_invD * ml);
  r = mb - r;
  p = r;
  q = r;

  u = r;

  real normb = sqrt((mb, mb));
  rtilde = r;

  if (normb == 0.0) {
    normb = 1;
  }

  if ((sqrt((r, r)) / normb) <= data_container->settings.solver.tolerance) {
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
    ShurProduct(phat, vhat);    // vhat = data_container->host_data.D_T *
                                // (data_container->host_data.M_invD * phat);
    alpha = rho_1 / (rtilde, vhat);
    q = u - alpha * vhat;
    uhat = (u + q);
    ml = ml + alpha * uhat;
    ShurProduct(uhat, qhat);    // qhat = data_container->host_data.D_T *
                                // (data_container->host_data.M_invD * uhat);
    r = r - alpha * qhat;
    rho_2 = rho_1;
    residual = (sqrt((r, r)) / normb);

    objective_value = GetObjectiveBlaze(ml, mb);
    AtIterationEnd(residual, objective_value, iter_hist.size());

    if (residual < data_container->settings.solver.tolerance) {
      break;
    }
  }
  Project(ml.data());

  return current_iteration;
}
