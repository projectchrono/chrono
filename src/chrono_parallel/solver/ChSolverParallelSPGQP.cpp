#include "chrono_parallel/solver/ChSolverParallel.h"

using namespace chrono;

ChSolverParallelSPGQP::ChSolverParallelSPGQP() : ChSolverParallel() {}

void ChSolverParallelSPGQP::UpdateR() {
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

uint ChSolverParallelSPGQP::Solve(ChShurProduct& ShurProduct,
                                  ChProjectConstraints& Project,
                                  const uint max_iter,
                                  const uint size,
                                  const DynamicVector<real>& r,
                                  DynamicVector<real>& gamma) {
    if (size == 0) {
        return 0;
    }

    real& lastgoodres = data_manager->measures.solver.residual;
    real& objective_value = data_manager->measures.solver.objective_value;
    real sigma_min = 0.1;
    real sigma_max = 0.9999;
    real gam = .1;
    int m = 10;
    real gdiff = 1.0 / pow(size, 2.0);
    lastgoodres = 10e30;
    f_hist.resize(max_iter + 1);
    temp.resize(size);
    Ad_k.resize(size);

    alpha = 0.0001;
    if (data_manager->settings.solver.cache_step_length == true) {
        if (data_manager->settings.solver.solver_mode == SolverMode::NORMAL) {
            alpha = data_manager->measures.solver.normal_apgd_step_length;
        } else if (data_manager->settings.solver.solver_mode == SolverMode::SLIDING) {
            alpha = data_manager->measures.solver.sliding_apgd_step_length;
        } else if (data_manager->settings.solver.solver_mode == SolverMode::SPINNING) {
            alpha = data_manager->measures.solver.spinning_apgd_step_length;
        } else if (data_manager->settings.solver.solver_mode == SolverMode::BILATERAL) {
            alpha = data_manager->measures.solver.bilateral_apgd_step_length;
        } else {
            alpha = 0.0001;
        }
    } else if (data_manager->settings.solver.use_power_iteration) {
        data_manager->measures.solver.lambda_max =
            LargestEigenValue(ShurProduct, temp, data_manager->measures.solver.lambda_max);
        alpha = 1.95 / data_manager->measures.solver.lambda_max;
    }
    x = gamma;
    x_candidate = gamma;
    ShurProduct(x, temp);
    g = temp - r;

    f_hist[0] = (0.5 * (g - r, x));

    for (current_iteration = 0; current_iteration < (signed)max_iter; current_iteration++) {
        temp = x - alpha * g;
        Project(temp.data());
        // g_alpha = 1.0 / alpha * (x - temp);

        // printf("||g_alpha||: %f \n", Sqrt((g_alpha, g_alpha)));
        //        if (Sqrt((g_alpha, g_alpha)) < data_manager->settings.solver.tolerance) {
        //            break;
        //        }

        d_k = temp - x;

        real max_compare = -10e29;
        for (int h = 1; h <= Min(current_iteration, m); h++) {
            real compare = f_hist[current_iteration - h];
            max_compare = Max(max_compare, compare);
        }

        f_max = max_compare;
        ShurProduct(d_k, Ad_k);
        real Ad_k_dot_d_k = (Ad_k, d_k);

        xi = (f_max - f_hist[current_iteration]) / Ad_k_dot_d_k;
        beta_bar = -(g, d_k) / Ad_k_dot_d_k;
        beta_tilde = gam * beta_bar + Sqrt(gam * gam * beta_bar * beta_bar + 2 * xi);
        beta_k = Min(sigma_max, beta_tilde);
        x = x + beta_k * d_k;
        g = g + beta_k * Ad_k;
        f_hist[current_iteration + 1] = (0.5 * (g - r, x));
        alpha = (d_k, d_k) / (Ad_k_dot_d_k);

        temp = x - gdiff * g;
        Project(temp.data());
        temp = (x - temp) / (-gdiff);

        real g_proj_norm = Sqrt((temp, temp));
        if (g_proj_norm < lastgoodres) {
            lastgoodres = g_proj_norm;
            objective_value = f_hist[current_iteration + 1];
            x_candidate = x;
        }
        // printf("R O [%f %f] \n", lastgoodres, objective_value);
        AtIterationEnd(lastgoodres, objective_value);

        if (lastgoodres < data_manager->settings.solver.tol_speed) {
            break;
        }
    }

    // printf("TIME: [%f %f %f %f]\n", t1(), t2(), t3(), t4());
    if (data_manager->settings.solver.solver_mode == SolverMode::NORMAL) {
        data_manager->measures.solver.normal_apgd_step_length = alpha;
    } else if (data_manager->settings.solver.solver_mode == SolverMode::SLIDING) {
        data_manager->measures.solver.sliding_apgd_step_length = alpha;
    } else if (data_manager->settings.solver.solver_mode == SolverMode::SPINNING) {
        data_manager->measures.solver.spinning_apgd_step_length = alpha;
    } else if (data_manager->settings.solver.solver_mode == SolverMode::BILATERAL) {
        data_manager->measures.solver.bilateral_apgd_step_length = alpha;
    }
    gamma = x_candidate;

    data_manager->system_timer.stop("ChSolverParallel_Solve");
    return current_iteration;
}
