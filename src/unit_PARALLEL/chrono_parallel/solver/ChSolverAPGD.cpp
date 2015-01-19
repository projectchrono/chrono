#include "chrono_parallel/solver/ChSolverAPGD.h"
#include <blaze/math/CompressedVector.h>
using namespace chrono;

void ChSolverAPGD::SetAPGDParams(real theta_k, real shrink, real grow) {
  init_theta = theta_k;
  step_shrink = shrink;
  step_grow = grow;
}

real ChSolverAPGD::Res4(blaze::DynamicVector<real>& mg_tmp2, blaze::DynamicVector<real>& x, blaze::DynamicVector<real>& temp) {
  real gdiff = 1e-6;
  real sum = 0;
  temp = x - gdiff * mg_tmp2;
  Project(temp.data());
  temp = (-1.0f / (gdiff)) * (temp - x);
  sum = (temp, temp);
  return sqrt(sum);
}


#define SHUR(x) (data_container->host_data.D_T * ( data_container->host_data.M_invD * x ) + data_container->host_data.E * x)

uint ChSolverAPGD::SolveAPGD(const uint max_iter, const uint size, const blaze::DynamicVector<real>& r, blaze::DynamicVector<real>& gamma) {
  real& residual = data_container->measures.solver.residual;
  real& objective_value = data_container->measures.solver.objective_value;
  custom_vector<real>& iter_hist = data_container->measures.solver.iter_hist;

  blaze::DynamicVector<real> one(size, 1.0);
  data_container->system_timer.start("ChSolverParallel_Solve");

  N_gamma_new.resize(size);
  temp.resize(size);
  g.resize(size);
  gamma_new.resize(size);
  y.resize(size);

  residual = 10e30;

  theta = init_theta;
  theta_new = theta;
  beta_new = 0.0;
  mb_tmp_norm = 0, mg_tmp_norm = 0;
  obj1 = 0.0, obj2 = 0.0;
  dot_mg_ms = 0, norm_ms = 0;
  delta_obj = 1e8;

  // Is the initial projection necessary?
  // Project(gamma.data());
  // gamma_hat = gamma;
  // ShurProduct(gamma, mg);
  // mg = mg - r;

  temp = gamma - one;
  L = sqrt((real)(temp, temp));
  temp = SHUR(temp); //ShurProduct(temp, temp);
  L = L == 0 ? 1 : L;
  L = sqrt((double)(temp, temp)) / L;

  t = 1.0 / L;
  y = gamma;

  for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {

    //ShurProduct(y, g);
    g = SHUR(y) - r;
    gamma_new = y - t * g;

    Project(gamma_new.data());

    N_gamma_new = SHUR(gamma_new); //ShurProduct(mx, temp_N_mx);
    obj1 = 0.5 * (gamma_new, N_gamma_new) - (gamma_new, r);

    //ShurProduct(y, temp);
    obj2 = 0.5 * (y, SHUR(y)) - (y, r);

    temp = gamma_new - y;
    dot_mg_ms = (g, temp);
    norm_ms = (temp, temp);

    while (obj1 > obj2 + dot_mg_ms + 0.5 * L * norm_ms) {
      L = 2.0 * L;
      t = 1.0 / L;
      gamma_new = y - t * g;
      Project(gamma_new.data());
      N_gamma_new = SHUR(gamma_new);       //ShurProduct(mx, temp_N_mx);
      obj1 = 0.5 * (gamma_new, N_gamma_new) - (gamma_new, r);
      temp = gamma_new - y;
      dot_mg_ms = (g, temp);
      norm_ms = (temp, temp);
    }
    theta_new = (-pow(theta, 2.0) + theta * sqrt(pow(theta, 2.0) + 4.0)) / 2.0;
    beta_new = theta * (1.0 - theta) / (pow(theta, 2.0) + theta_new);

    temp = gamma_new - gamma;
    y = beta_new * temp + gamma_new;
    real dot_mg_ms = (g, temp);

    N_gamma_new = N_gamma_new - r;
    real res = Res4(N_gamma_new, gamma, temp);

    if (res < residual) {
      residual = res;
      gamma_hat = gamma_new;
    }

    AtIterationEnd(residual, objective_value, iter_hist.size());
    if (residual < data_container->settings.solver.tolerance) {
      break;
    }

    if (dot_mg_ms > 0) {
      y = gamma_new;
      theta_new = 1.0;
    }

    L = 0.9 * L;
    t = 1.0 / L;
    step_grow = 2.0;
    theta = theta_new;

    gamma = gamma_new;
    //objective_value = (gamma_hat, mso);    // maxdeltalambda = GetObjectiveBlaze(gamma_hat, r);




//    if (data_container->settings.solver.test_objective) {
//      if (objective_value <= data_container->settings.solver.tolerance_objective) {
//        break;
//      }
//    } else {
//    }
  }

  gamma = gamma_hat;

  data_container->system_timer.stop("ChSolverParallel_Solve");
  return current_iteration;
}
