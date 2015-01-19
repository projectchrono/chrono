#include "chrono_parallel/solver/ChSolverAPGD.h"
#include <blaze/math/CompressedVector.h>
using namespace chrono;

void ChSolverAPGD::SetAPGDParams(real theta_k, real shrink, real grow) {
  init_theta_k = theta_k;
  step_shrink = shrink;
  step_grow = grow;
}

real ChSolverAPGD::Res4(const int SIZE, blaze::DynamicVector<real>& mg_tmp2, blaze::DynamicVector<real>& x, blaze::DynamicVector<real>& mb_tmp) {
  real gdiff = 1e-6;
  real sum = 0;
  mb_tmp = -gdiff * mg_tmp2 + x;
  Project(mb_tmp.data());
  mb_tmp = (-1.0f / (gdiff)) * (mb_tmp - x);
  sum = (mb_tmp, trans(mb_tmp));
  return sqrt(sum);
}

uint ChSolverAPGD::SolveAPGD(const uint max_iter, const uint size, const blaze::DynamicVector<real>& r, blaze::DynamicVector<real>& gamma) {
  real& residual = data_container->measures.solver.residual;
  real& objective_value = data_container->measures.solver.objective_value;
  custom_vector<real>& iter_hist = data_container->measures.solver.iter_hist;

  blaze::DynamicVector<real> one(size, 1.0);
  data_container->system_timer.start("ChSolverParallel_Solve");

  ms.resize(size);
  mg_tmp2.resize(size);
  temp.resize(size);
  g.resize(size);
  mg.resize(size);
  mx.resize(size);
  y.resize(size);
  mso.resize(size);

  residual = 10e30;

  theta_k = init_theta_k;
  theta_k1 = theta_k;
  beta_k1 = 0.0;
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
  ShurProduct(temp, temp);
  L = L == 0 ? 1 : L;
  L = sqrt((double)(temp, temp)) / L;

  t = 1.0 / L;
  y = gamma;

  for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {

    ShurProduct(y, g);
    g = g - r;
    mx = y - t * g;

    Project(mx.data());

    ShurProduct(mx, temp);

    obj1 = 0.5 *(mx, temp) - (mx , r);
    obj2 = 0.5 * (y, temp) - (y, r);

   // ms = .5 * mg_tmp1 - r;
    ms = mx - y;
    dot_mg_ms = (mg,ms);
    norm_ms = (ms,ms);

    while (obj1 > obj2 + dot_mg_ms + 0.5 * L * norm_ms) {
      L = 2.0 * L;
      t = 1.0 / L;
      mx = -t * mg + y;
      Project(mx.data());

      ShurProduct(mx, temp);
      mso = .5 * temp - r;
      obj1 = (mx, trans(mso));
      ms = mx - y;
      dot_mg_ms = (mg, trans(ms));
      norm_ms = (ms, trans(ms));
    }
    theta_k1 = (-pow(theta_k, 2) + theta_k * sqrt(pow(theta_k, 2) + 4)) / 2.0;
    beta_k1 = theta_k * (1.0 - theta_k) / (pow(theta_k, 2) + theta_k1);

    ms = mx - gamma;
    y = beta_k1 * ms + mx;
    real dot_mg_ms = (mg, trans(ms));

    if (dot_mg_ms > 0) {
      y = mx;
      theta_k1 = 1.0;
    }
    L = 0.9 * L;
    t = 1.0 / L;
    gamma = mx;
    step_grow = 2.0;
    theta_k = theta_k1;
    // if (current_iteration % 2 == 0) {
    mg_tmp2 = temp - r;
    real g_proj_norm = Res4(num_unilaterals, mg_tmp2, gamma, temp);

    if (num_bilaterals > 0) {
      real resid_bilat = -1;
      for (int i = num_unilaterals; i < gamma.size(); i++) {
        resid_bilat = std::max(resid_bilat, std::abs(mg_tmp2[i]));
      }
      g_proj_norm = std::max(g_proj_norm, resid_bilat);
    }

    bool update = false;
    if (g_proj_norm < residual) {
      residual = g_proj_norm;
      gamma_hat = gamma;
      objective_value = (gamma_hat, mso);    // maxdeltalambda = GetObjectiveBlaze(gamma_hat, r);
      update = true;
    }

    AtIterationEnd(residual, objective_value, iter_hist.size());
    if (data_container->settings.solver.test_objective) {
      if (objective_value <= data_container->settings.solver.tolerance_objective) {
        break;
      }
    } else {
      if (residual < data_container->settings.solver.tolerance) {
        break;
      }
    }
  }

  gamma = gamma_hat;

  data_container->system_timer.stop("ChSolverParallel_Solve");
  return current_iteration;
}
