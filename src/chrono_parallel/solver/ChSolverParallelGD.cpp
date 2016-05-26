#include "chrono_parallel/solver/ChSolverParallelGD.h"

using namespace chrono;

uint ChSolverParallelGD::SolveGD(const uint max_iter,
                                 const uint size,
                                 DynamicVector<real>& mb,
                                 DynamicVector<real>& ml) {
  real& residual = data_manager->measures.solver.residual;
  real& objective_value = data_manager->measures.solver.objective_value;

  real eps = data_manager->settings.step_size;
  r.resize(size);

  ShurProduct(ml, r);  // r = data_manager->host_data.D_T *
                       // (data_manager->host_data.M_invD * ml);
  r = mb - r;
  real resold = 1, resnew, normb = Sqrt((mb, mb));
  if (normb == 0.0) {
    normb = 1;
  };
  for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
    ml = ml + eps * r;
    ShurProduct(ml, r);
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
