#include "chrono_parallel/solver/ChSolverCG.h"

using namespace chrono;

uint ChSolverCG::SolveCG(const uint max_iter, const uint size, blaze::DynamicVector<real>& mb, blaze::DynamicVector<real>& ml) {
  real& residual = data_container->measures.solver.residual;
  real& objective_value = data_container->measures.solver.objective_value;
  custom_vector<real>& iter_hist = data_container->measures.solver.iter_hist;

  r.resize(size), Ap.resize(size);

  real rsold, alpha, rsnew = 0, normb = sqrt((mb, mb));
  if (normb == 0.0) {
    normb = 1;
  }
  ShurProduct(ml, r);    // r = data_container->host_data.D_T *
                         // (data_container->host_data.M_invD * ml);
  p = r = mb - r;
  rsold = (r, r);
  normb = 1.0 / normb;
  if (sqrt(rsold) * normb <= data_container->settings.solver.tolerance) {
    return 0;
  }
  for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
    ShurProduct(p, Ap);    // Ap = data_container->host_data.D_T *
                           // (data_container->host_data.M_invD * p);
    alpha = rsold / (p, Ap);
    rsnew = 0;
    ml = alpha * p + ml;
    r = -alpha * Ap + r;
    rsnew = (r, r);

    residual = sqrt(rsnew) * normb;
    if (residual < data_container->settings.solver.tolerance) {
      break;
    }
    p = rsnew / rsold * p + r;
    rsold = rsnew;

    objective_value = GetObjectiveBlaze(ml, mb);
    AtIterationEnd(residual, objective_value);
  }
  Project(ml.data());

  return current_iteration;
}
