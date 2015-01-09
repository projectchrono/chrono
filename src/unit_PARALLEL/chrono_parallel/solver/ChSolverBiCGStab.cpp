#include "chrono_parallel/solver/ChSolverBiCGStab.h"

using namespace chrono;

uint ChSolverBiCGStab::SolveBiCGStab(const uint max_iter, const uint size, blaze::DynamicVector<real>& mb, blaze::DynamicVector<real>& ml) {
  real& residual = data_container->measures.solver.residual;
  real& objective_value = data_container->measures.solver.objective_value;
  custom_vector<real>& iter_hist = data_container->measures.solver.iter_hist;


  real rho_1, rho_2, alpha = 1, beta, omega = 1;

  r.resize(size);
  t.resize(size);
  v.resize(size);
  real normb = sqrt((mb, mb));

  ShurProduct(ml, r);    // r = data_container->host_data.D_T *
                         // (data_container->host_data.M_invD * ml);
  p = r = mb - r;
  rtilde = r;

  if (normb == 0.0) {
    normb = 1;
  }

  if ((residual = sqrt((r, r)) / normb) <= data_container->settings.solver.tolerance) {
    return 0;
  }

  for (current_iteration = 0; current_iteration <= max_iter; current_iteration++) {
    rho_1 = (rtilde, r);

    if (rho_1 == 0) {
      break;
    }

    if (current_iteration > 0) {
      beta = (rho_1 / rho_2) * (alpha / omega);
      p = r + beta * (p - omega * v);
    }

    phat = p;
    ShurProduct(phat, v);    // v = data_container->host_data.D_T *
                             // (data_container->host_data.M_invD * phat);
    alpha = rho_1 / (rtilde, v);
    s = r - alpha * v;    // SEAXPY(-alpha,v,r,s);//
    residual = sqrt((s, s)) / normb;

    if (residual < data_container->settings.solver.tolerance) {
      ml = ml + alpha * phat;
      break;
    }

    shat = s;
    ShurProduct(shat, t);    // t = data_container->host_data.D_T *
                             // (data_container->host_data.M_invD * shat);
    omega = (t, s) / (t, t);
    ml = ml + alpha * phat + omega * shat;
    r = s - omega * t;
    rho_2 = rho_1;
    residual = sqrt((r, r)) / normb;

    objective_value = GetObjectiveBlaze(ml, mb);
    AtIterationEnd(residual, objective_value, iter_hist.size());

    if (residual < data_container->settings.solver.tolerance || omega == 0) {
      break;
    }
  }

  return current_iteration;
}
