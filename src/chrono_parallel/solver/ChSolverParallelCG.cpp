#include "chrono_parallel/solver/ChSolverParallelCG.h"

using namespace chrono;

uint ChSolverParallelCG::SolveCG(const uint max_iter,
                                 const uint size,
                                 DynamicVector<real>& mb,
                                 DynamicVector<real>& ml) {
  real& residual = data_manager->measures.solver.residual;
  real& objective_value = data_manager->measures.solver.objective_value;

  r.resize(size), Ap.resize(size);

  real rsold, alpha, rsnew = 0, normb = Sqrt((mb, mb));
  if (normb == 0.0) {
    normb = 1;
  }
  ShurProduct(ml, r);  // r = data_manager->host_data.D_T *
                       // (data_manager->host_data.M_invD * ml);
  p = r = mb - r;
  rsold = (r, r);
  normb = 1.0 / normb;
  if (Sqrt(rsold) * normb <= data_manager->settings.solver.tolerance) {
    return 0;
  }
  for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
    ShurProduct(p, Ap);  // Ap = data_manager->host_data.D_T *
                         // (data_manager->host_data.M_invD * p);
    alpha = rsold / (p, Ap);
    rsnew = 0;
    ml = alpha * p + ml;
    r = -alpha * Ap + r;
    rsnew = (r, r);

    residual = Sqrt(rsnew) * normb;
    if (residual < data_manager->settings.solver.tolerance) {
      break;
    }
    p = rsnew / rsold * p + r;
    rsold = rsnew;

    objective_value = GetObjective(ml, mb);
    AtIterationEnd(residual, objective_value);
  }
  Project(ml.data());

  return current_iteration;
}
