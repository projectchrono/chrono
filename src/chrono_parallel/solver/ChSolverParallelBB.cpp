#include "chrono_parallel/solver/ChSolverParallel.h"

using namespace chrono;

ChSolverParallelBB::ChSolverParallelBB() : ChSolverParallel() {}

void ChSolverParallelBB::UpdateR() {
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

uint ChSolverParallelBB::Solve(ChShurProduct& ShurProduct,
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

    // ChTimer<> t1, t2, t3, t4;
    // t1.start();

    temp.resize(size);
    ml.resize(size);
    mg.resize(size);
    mg_p.resize(size);
    ml_candidate.resize(size);
    ms.resize(size);
    my.resize(size);
    mdir.resize(size);
    ml_p.resize(size);

    temp = 0;
    ml = 0;
    mg = 0;
    mg_p = 0;
    ml_candidate = 0;
    ms = 0;
    my = 0;
    mdir = 0;
    ml_p = 0;

    // Tuning of the spectral gradient search
    real a_min = 1e-13;
    real a_max = 1e13;
    real sigma_min = 0.1;
    real sigma_max = 0.9;

    real alpha = 0.0001;
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
    real gmma = 1e-4;
    real gdiff = 1.0 / pow(size, 2.0);
    bool do_preconditioning = false;
    real neg_BB1_fallback = 0.11;
    real neg_BB2_fallback = 0.12;
    ml = gamma;
    lastgoodres = 10e30;
    real lastgoodfval = 10e30;
    ml_candidate = ml;
    ShurProduct(ml, temp);
    mg = temp - r;
    mg_p = mg;

    real mf_p = 0;
    real mf = 1e29;
    int n_armijo = 10;
    int max_armijo_backtrace = 3;
    std::vector<real> f_hist;
    // t1.stop();

    for (current_iteration = 0; current_iteration < (signed)max_iter; current_iteration++) {
        // t2.start();
        temp = (ml - alpha * mg);
        Project(temp.data());
        mdir = temp - ml;

        real dTg = (mdir, mg);
        real lambda = 1.0;
        int n_backtracks = 0;
        bool armijo_repeat = true;
        // t2.stop();
        // t3.start();
        while (armijo_repeat) {
            ml_p = ml + lambda * mdir;

            ShurProduct(ml_p, temp);
            mg_p = temp - r;
            mf_p = (ml_p, 0.5 * temp - r);

            f_hist.push_back(mf_p);

            real max_compare = 10e29;
            for (int h = 1; h <= Min(current_iteration, n_armijo); h++) {
                real compare = f_hist[current_iteration - h] + gmma * lambda * dTg;
                if (compare > max_compare)
                    max_compare = compare;
            }
            if (mf_p > max_compare) {
                armijo_repeat = true;
                if (current_iteration > 0)
                    mf = f_hist[current_iteration - 1];
                real lambdanew = -lambda * lambda * dTg / (2 * (mf_p - mf - lambda * dTg));
                lambda = Max(sigma_min * lambda, Min(sigma_max * lambda, lambdanew));
                printf("Repeat Armijo, new lambda = %f \n", lambda);
            } else {
                armijo_repeat = false;
            }
            n_backtracks = n_backtracks + 1;
            if (n_backtracks > max_armijo_backtrace)
                armijo_repeat = false;
        }
        // t3.stop();
        // t4.start();
        ms = ml_p - ml;
        my = mg_p - mg;
        ml = ml_p;
        mg = mg_p;

        if (current_iteration % 2 == 0) {
            real sDs = (ms, ms);
            real sy = (ms, my);
            if (sy <= 0) {
                alpha = neg_BB1_fallback;
            } else {
                alpha = Min(a_max, Max(a_min, sDs / sy));
            }
        } else {
            real sy = (ms, my);
            real yDy = (my, my);
            if (sy <= 0) {
                alpha = neg_BB2_fallback;
            } else {
                alpha = Min(a_max, Max(a_min, sy / yDy));
            }
        }
        temp = ml - gdiff * mg;
        Project(temp.data());
        temp = (ml - temp) / (-gdiff);

        real g_proj_norm = Sqrt((temp, temp));
        if (g_proj_norm < lastgoodres) {
            lastgoodres = g_proj_norm;
            objective_value = mf_p;
            ml_candidate = ml;
        }

        AtIterationEnd(lastgoodres, objective_value);

		if (data_manager->settings.solver.test_objective) {
			if (objective_value <= data_manager->settings.solver.tolerance_objective) {
				break;
			}
		}
		else {
			if (lastgoodres < data_manager->settings.solver.tol_speed) {
				break;
			}
		}


        // t4.stop();
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
    gamma = ml_candidate;
    data_manager->system_timer.stop("ChSolverParallel_Solve");
    return current_iteration;
}
