#include "chrono_parallel/solver/ChSolverParallelAPGD.h"
#include <blaze/math/CompressedVector.h>
using namespace chrono;

ChSolverParallelAPGD::ChSolverParallelAPGD()
    : ChSolverParallel(),
      mg_tmp_norm(0),
      mb_tmp_norm(0),
      obj1(0),
      obj2(0),
      norm_ms(0),
      dot_g_temp(0),
      theta(1),
      theta_new(0),
      beta_new(0),
      t(0),
      L(0),
      g_diff(0) {
}

void ChSolverParallelAPGD::UpdateR() {
  const CompressedMatrix<real>& D_n_T = data_manager->host_data.D_n_T;
  const DynamicVector<real>& M_invk = data_manager->host_data.M_invk;
  const DynamicVector<real>& b = data_manager->host_data.b;
  DynamicVector<real>& R = data_manager->host_data.R;
  DynamicVector<real>& s = data_manager->host_data.s;

  uint num_contacts = data_manager->num_rigid_contacts;

  s.resize(data_manager->num_rigid_contacts);
  reset(s);

  rigid_rigid->Build_s();

  ConstSubVectorType b_n = blaze::subvector(b, 0, num_contacts);
  SubVectorType R_n = blaze::subvector(R, 0, num_contacts);
  SubVectorType s_n = blaze::subvector(s, 0, num_contacts);

  R_n = -b_n - D_n_T * M_invk - s_n;
}

uint ChSolverParallelAPGD::SolveAPGD(const uint max_iter,
                                     const uint size,
                                     const DynamicVector<real>& r,
                                     DynamicVector<real>& gamma) {
  real& residual = data_manager->measures.solver.residual;
  real& objective_value = data_manager->measures.solver.objective_value;

  DynamicVector<real> one(size, 1.0);
  data_manager->system_timer.start("ChSolverParallel_Solve");
  gamma_hat.resize(size);
  N_gamma_new.resize(size);
  temp.resize(size);
  g.resize(size);
  gamma_new.resize(size);
  y.resize(size);

  residual = 10e30;
  g_diff = 1.0 / pow(size, 2.0);

  theta = 1;
  theta_new = theta;
  beta_new = 0.0;
  mb_tmp_norm = 0, mg_tmp_norm = 0;
  obj1 = 0.0, obj2 = 0.0;
  dot_g_temp = 0, norm_ms = 0;

  // Is the initial projection necessary?
  // Project(gamma.data());
  // gamma_hat = gamma;
  // ShurProduct(gamma, mg);
  // mg = mg - r;

  temp = gamma - one;
  real norm_temp = Sqrt((real)(temp, temp));

  // If gamma is one temp should be zero, in that case set L to one
  // We cannot divide by 0
  if (norm_temp == 0) {
    L = 1.0;
  } else {
    // If the N matrix is zero for some reason, temp will be zero
    ShurProduct(temp, temp);
    // If temp is zero then L will be zero
    L = Sqrt((real)(temp, temp)) / norm_temp;
  }
  // When L is zero the step length can't be computed, in this case just return
  // If the N is indeed zero then solving doesn't make sense
  if (L == 0) {
    // For certain simulations returning here will not perform any iterations
    // even when there are contacts that aren't resolved. Changed it from return 0
    // to L=t=1;
    // return 0;
    L = t = 1;
  } else {
    // Compute the step size
    t = 1.0 / L;
  }
  y = gamma;
  // If no iterations are performed or the residual is NAN (which is shouldnt be)
  // make sure that gamma_hat has something inside of it. Otherwise gamma will be
  // overwritten with a vector of zero size
  gamma_hat = gamma;

  for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
    ShurProduct(y, g);
    g = g - r;
    gamma_new = y - t * g;

    Project(gamma_new.data());

    ShurProduct(gamma_new, N_gamma_new);
    obj1 = 0.5 * (gamma_new, N_gamma_new) - (gamma_new, r);

    ShurProduct(y, temp);
    obj2 = 0.5 * (y, temp) - (y, r);

    temp = gamma_new - y;
    dot_g_temp = (g, temp);
    norm_ms = (temp, temp);
    while (obj1 > obj2 + dot_g_temp + 0.5 * L * norm_ms) {
      L = 2.0 * L;
      t = 1.0 / L;
      gamma_new = y - t * g;
      Project(gamma_new.data());
      ShurProduct(gamma_new, N_gamma_new);
      obj1 = 0.5 * (gamma_new, N_gamma_new) - (gamma_new, r);
      temp = gamma_new - y;
      dot_g_temp = (g, temp);
      norm_ms = (temp, temp);
    }
    theta_new = (-pow(theta, 2.0) + theta * Sqrt(pow(theta, 2.0) + 4.0)) / 2.0;
    beta_new = theta * (1.0 - theta) / (pow(theta, 2.0) + theta_new);

    temp = gamma_new - gamma;
    y = beta_new * temp + gamma_new;
    dot_g_temp = (g, temp);

    // Compute the residual
    temp = gamma_new - g_diff * (N_gamma_new - r);
    real temp_dota = (real)(temp, temp);
    // ಠ_ಠ THIS PROJECTION IS IMPORTANT! (╯°□°)╯︵ ┻━┻
    // If turned off the residual will be very incorrect! Turning it off can cause the solver to effectively use the
    // solution found in the first step because the residual never get's smaller. (You can convince yourself of this by
    // looking at the objective function value and watch it decrease while the residual and the current solution remain
    // the same.)
    Project(temp.data());
    temp = (1.0 / g_diff) * (gamma_new - temp);
    real temp_dotb = (real)(temp, temp);
    real res = Sqrt(temp_dotb);

    if (res < residual) {
      residual = res;
      gamma_hat = gamma_new;
    }

    // Compute the objective value
    temp = 0.5 * N_gamma_new - r;
    objective_value = (gamma_new, temp);

    AtIterationEnd(residual, objective_value);

    if (data_manager->settings.solver.test_objective) {
      if (objective_value <= data_manager->settings.solver.tolerance_objective) {
        break;
      }
    } else {
      if (residual < data_manager->settings.solver.tol_speed) {
        break;
      }
    }

    if (dot_g_temp > 0) {
      y = gamma_new;
      theta_new = 1.0;
    }

    L = 0.9 * L;
    t = 1.0 / L;
    theta = theta_new;
    gamma = gamma_new;

    if (data_manager->settings.solver.update_rhs) {
      UpdateR();
    }
  }

  gamma = gamma_hat;

  data_manager->system_timer.stop("ChSolverParallel_Solve");
  return current_iteration;
}
