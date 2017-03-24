#include "chrono_parallel/solver/ChSolverParallel.h"

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
      g_diff(0) {}

void ChSolverParallelAPGD::UpdateR() {
    const SubMatrixType& D_n_T = _DNT_;
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

    R_n = -b_n - D_n_T * M_invk + s_n;
}

uint ChSolverParallelAPGD::Solve(ChShurProduct& ShurProduct,
                                 ChProjectConstraints& Project,
                                 const uint max_iter,
                                 const uint size,
                                 const DynamicVector<real>& r,
                                 DynamicVector<real>& gamma) {
    if (size == 0) {
        return 0;
    }

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
    t = L = 1.0;
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
    if (data_manager->settings.solver.cache_step_length == true) {
        if (data_manager->settings.solver.solver_mode == SolverMode::NORMAL) {
            L = data_manager->measures.solver.normal_apgd_step_length;
        } else if (data_manager->settings.solver.solver_mode == SolverMode::SLIDING) {
            L = data_manager->measures.solver.sliding_apgd_step_length;
        } else if (data_manager->settings.solver.solver_mode == SolverMode::SPINNING) {
            L = data_manager->measures.solver.spinning_apgd_step_length;
        } else if (data_manager->settings.solver.solver_mode == SolverMode::BILATERAL) {
            L = data_manager->measures.solver.bilateral_apgd_step_length;
        } else {
            L = 1.0;
        }
    } else if (data_manager->settings.solver.use_power_iteration) {
        data_manager->measures.solver.lambda_max =
            LargestEigenValue(ShurProduct, temp, data_manager->measures.solver.lambda_max);
        L = data_manager->measures.solver.lambda_max;
    } else {
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
    }

    t = 1.0 / L;
    y = gamma;
    // If no iterations are performed or the residual is NAN (which is shouldnt be)
    // make sure that gamma_hat has something inside of it. Otherwise gamma will be
    // overwritten with a vector of zero size
    gamma_hat = gamma;

    for (current_iteration = 0; current_iteration < (signed)max_iter; current_iteration++) {
        ShurProduct(y, temp);
        // ShurProduct(y, g);
        g = temp - r;
        gamma_new = y - t * g;
        Project(gamma_new.data());
        ShurProduct(gamma_new, N_gamma_new);
        obj2 = (y, 0.5 * temp - r);
        temp = gamma_new - y;
        while ((gamma_new, 0.5 * N_gamma_new - r) > obj2 + (g + 0.5 * L * temp, temp)) {
            L = 2.0 * L;
            t = 1.0 / L;
            gamma_new = y - t * g;
            Project(gamma_new.data());
            ShurProduct(gamma_new, N_gamma_new);
            obj1 = (gamma_new, 0.5 * N_gamma_new - r);
            temp = gamma_new - y;
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
        // solution found in the first step because the residual never get's smaller. (You can convince yourself of this
        // by
        // looking at the objective function value and watch it decrease while the residual and the current solution
        // remain
        // the same.)
        Project(temp.data());
        temp = (1.0 / g_diff) * (gamma_new - temp);
        real temp_dotb = (real)(temp, temp);
        real res = Sqrt(temp_dotb);

        if (res < residual) {
            residual = res;
            gamma_hat = gamma_new;

            // Compute the objective value
            temp = 0.5 * N_gamma_new - r;
            objective_value = (gamma_new, temp);
        }

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
    if (data_manager->settings.solver.solver_mode == SolverMode::NORMAL) {
        data_manager->measures.solver.normal_apgd_step_length = L;
    } else if (data_manager->settings.solver.solver_mode == SolverMode::SLIDING) {
        data_manager->measures.solver.sliding_apgd_step_length = L;
    } else if (data_manager->settings.solver.solver_mode == SolverMode::SPINNING) {
        data_manager->measures.solver.spinning_apgd_step_length = L;
    } else if (data_manager->settings.solver.solver_mode == SolverMode::BILATERAL) {
        data_manager->measures.solver.bilateral_apgd_step_length = L;
    }
    gamma = gamma_hat;

    data_manager->system_timer.stop("ChSolverParallel_Solve");
    return current_iteration;
}
