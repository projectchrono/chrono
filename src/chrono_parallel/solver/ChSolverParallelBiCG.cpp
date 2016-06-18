#include "chrono_parallel/solver/ChSolverParallelBiCG.h"

using namespace chrono;

uint ChSolverParallelBiCG::SolveBiCG(const uint max_iter,
                                     const uint size,
                                     DynamicVector<real>& mb,
                                     DynamicVector<real>& ml) {
  real& residual = data_manager->measures.solver.residual;
  real& objective_value = data_manager->measures.solver.objective_value;


  q.resize(size), qtilde.resize(size), r.resize(size);
  real normb = Sqrt((mb, mb));
  ShurProduct(ml, r);  // r = data_manager->host_data.D_T *
                       // (data_manager->host_data.M_invD * ml);
  r = mb - r;

  rtilde = r;

  if (normb == 0.0) {
    normb = 1;
  }

  if ((residual = Sqrt((r, r)) / normb) <= data_manager->settings.solver.tolerance) {
    return 0;
  }

  for (current_iteration = 0; current_iteration <= max_iter; current_iteration++) {
    z = (r);
    ztilde = rtilde;
    rho_1 = (z, rtilde);

    if (rho_1 == 0) {
      break;
    }

    if (current_iteration == 0) {
      p = z;
      ptilde = ztilde;
    } else {
      beta = rho_1 / rho_2;
      p = z + beta * p;
      ptilde = ztilde + beta * ptilde;
    }

    ShurProduct(p, q);            // q = data_manager->host_data.D_T *
                                  // (data_manager->host_data.M_invD * p);
    ShurProduct(ptilde, qtilde);  // qtilde = data_manager->host_data.D_T *
                                  // (data_manager->host_data.M_invD *
                                  // ptilde);

    alpha = rho_1 / (ptilde, q);
    ml = ml + alpha * p;
    r = r - alpha * q;
    rtilde = rtilde - alpha * qtilde;
    rho_2 = rho_1;
    residual = Sqrt((r, r)) / normb;

    objective_value = GetObjective(ml, mb);
    AtIterationEnd(residual, objective_value);

    if (residual < data_manager->settings.solver.tolerance) {
      break;
    }
  }

  return current_iteration;
}
