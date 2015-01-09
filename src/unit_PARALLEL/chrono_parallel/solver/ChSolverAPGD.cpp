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

uint ChSolverAPGD::SolveAPGD(const uint max_iter, const uint size, const blaze::DynamicVector<real>& mb, blaze::DynamicVector<real>& ml) {
  real& residual = data_container->measures.solver.residual;
  real& objective_value = data_container->measures.solver.objective_value;
  custom_vector<real>& iter_hist = data_container->measures.solver.iter_hist;


  // data_container->system_timer.start("ChSolverParallel_solverA");
  blaze::DynamicVector<real> one(size, 1.0);
  data_container->system_timer.start("ChSolverParallel_Solve");
  ms.resize(size);
  mg_tmp2.resize(size);
  mb_tmp.resize(size);
  mg_tmp.resize(size);
  mg_tmp1.resize(size);
  mg.resize(size);
  mx.resize(size);
  my.resize(size);
  mso.resize(size);

  lastgoodres = 10e30;
  theta_k = init_theta_k;
  theta_k1 = theta_k;
  beta_k1 = 0.0;
  mb_tmp_norm = 0, mg_tmp_norm = 0;
  obj1 = 0.0, obj2 = 0.0;
  dot_mg_ms = 0, norm_ms = 0;
  delta_obj = 1e8;

  Project(ml.data());
  ml_candidate = ml;
  ShurProduct(ml, mg);
  mg = mg - mb;
  mb_tmp = ml - one;
  ShurProduct(mb_tmp, mg_tmp);

  mb_tmp_norm = sqrt((mb_tmp, trans(mb_tmp)));
  mg_tmp_norm = sqrt((mg_tmp, trans(mg_tmp)));

  if (mb_tmp_norm == 0) {
    L_k = 1;
  } else {
    L_k = mg_tmp_norm / mb_tmp_norm;
  }

  t_k = 1.0 / L_k;
  my = ml;
  mx = ml;

  for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {

    ShurProduct(my, mg_tmp1);

    mg = mg_tmp1 - mb;
    mx = -t_k * mg + my;

    Project(mx.data());
    ShurProduct(mx, mg_tmp);

    // mg_tmp2 = mg_tmp - mb;
    mso = .5 * mg_tmp - mb;
    obj1 = (mx, trans(mso));
    ms = .5 * mg_tmp1 - mb;
    obj2 = (my, trans(ms));
    ms = mx - my;
    dot_mg_ms = (mg, trans(ms));
    norm_ms = (ms, trans(ms));

    while (obj1 > obj2 + dot_mg_ms + 0.5 * L_k * norm_ms) {
      L_k = 2.0 * L_k;
      t_k = 1.0 / L_k;
      mx = -t_k * mg + my;
      Project(mx.data());

      ShurProduct(mx, mg_tmp);
      mso = .5 * mg_tmp - mb;
      obj1 = (mx, trans(mso));
      ms = mx - my;
      dot_mg_ms = (mg, trans(ms));
      norm_ms = (ms, trans(ms));

    }
    theta_k1 = (-pow(theta_k, 2) + theta_k * sqrt(pow(theta_k, 2) + 4)) / 2.0;
    beta_k1 = theta_k * (1.0 - theta_k) / (pow(theta_k, 2) + theta_k1);

    ms = mx - ml;
    my = beta_k1 * ms + mx;
    real dot_mg_ms = (mg, trans(ms));

    if (dot_mg_ms > 0) {
      my = mx;
      theta_k1 = 1.0;
    }
    L_k = 0.9 * L_k;
    t_k = 1.0 / L_k;
    ml = mx;
    step_grow = 2.0;
    theta_k = theta_k1;
    // if (current_iteration % 2 == 0) {
    mg_tmp2 = mg_tmp - mb;
    real g_proj_norm = Res4(num_unilaterals, mg_tmp2, ml, mb_tmp);

    if (num_bilaterals > 0) {
      real resid_bilat = -1;
      for (int i = num_unilaterals; i < ml.size(); i++) {
        resid_bilat = std::max(resid_bilat, std::abs(mg_tmp2[i]));
      }
      g_proj_norm = std::max(g_proj_norm, resid_bilat);
    }

    bool update = false;
    if (g_proj_norm < lastgoodres) {
      lastgoodres = g_proj_norm;
      ml_candidate = ml;
      objective_value = (ml_candidate, mso);    // maxdeltalambda = GetObjectiveBlaze(ml_candidate, mb);
      update = true;
    }

    residual = lastgoodres;

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

  ml = ml_candidate;

  data_container->system_timer.stop("ChSolverParallel_Solve");
  return current_iteration;
}
