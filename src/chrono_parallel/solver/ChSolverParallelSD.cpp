#include "chrono_parallel/solver/ChSolverParallelSD.h"

using namespace chrono;

uint ChSolverParallelSD::SolveSD(const uint max_iter,
                                 const uint size,
                                 DynamicVector<real>& mb,
                                 DynamicVector<real>& ml) {
  real& residual = data_manager->measures.solver.residual;
  real& objective_value = data_manager->measures.solver.objective_value;

  r.resize(size);
  temp.resize(size);

  ShurProduct(ml, r);  // r = data_manager->host_data.D_T *
                       // (data_manager->host_data.M_invD * ml);

  r = mb - r;
  real resold = 1, resnew, normb = Sqrt((mb, mb)), alpha;
  if (normb == 0.0) {
    normb = 1;
  }
  for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
    ShurProduct(r, temp);  // temp = data_manager->host_data.D_T *
                           // (data_manager->host_data.M_invD * r);
    alpha = (r, r) / (r, temp);
    ml = ml + alpha * r;
    ShurProduct(ml, r);  // r = data_manager->host_data.D_T *
                         // (data_manager->host_data.M_invD * ml);
    r = mb - r;
    resnew = Sqrt((ml, ml));
    residual = std::abs(resnew - resold);

    objective_value = GetObjective(ml, mb);
    AtIterationEnd(residual, objective_value);
    if (residual < data_manager->settings.solver.tolerance) {
      break;
    }
    resold = resnew;
  }
  Project(ml.data());

  return current_iteration;
}
