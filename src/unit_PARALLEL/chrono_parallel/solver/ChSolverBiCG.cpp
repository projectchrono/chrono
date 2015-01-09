#include "chrono_parallel/solver/ChSolverBiCG.h"

using namespace chrono;

uint ChSolverBiCG::SolveBiCG(const uint max_iter, const uint size, blaze::DynamicVector<real>& mb, blaze::DynamicVector<real>& ml) {
  real& residual = data_container->measures.solver.residual;
  real& objective_value = data_container->measures.solver.objective_value;
  custom_vector<real>& iter_hist = data_container->measures.solver.iter_hist;


  q.resize(size), qtilde.resize(size), r.resize(size);
  real normb = std::sqrt((mb, mb));
  ShurProduct(ml, r);    // r = data_container->host_data.D_T *
                         // (data_container->host_data.M_invD * ml);
  r = mb - r;

  rtilde = r;

  if (normb == 0.0) {
    normb = 1;
  }

  if ((residual = std::sqrt((r, r)) / normb) <= data_container->settings.solver.tolerance) {
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

    ShurProduct(p, q);              // q = data_container->host_data.D_T *
                                    // (data_container->host_data.M_invD * p);
    ShurProduct(ptilde, qtilde);    // qtilde = data_container->host_data.D_T *
                                    // (data_container->host_data.M_invD *
                                    // ptilde);

    alpha = rho_1 / (ptilde, q);
    ml = ml + alpha * p;
    r = r - alpha * q;
    rtilde = rtilde - alpha * qtilde;
    rho_2 = rho_1;
    residual = sqrt((r, r)) / normb;

    objective_value = GetObjectiveBlaze(ml, mb);
    AtIterationEnd(residual, objective_value, iter_hist.size());

    if (residual < data_container->settings.solver.tolerance) {
      break;
    }
  }

  return current_iteration;
}
