#include "chrono_parallel/solver/ChSolverParallel.h"

using namespace chrono;

real ChSolverParallelAPGDREF::Res4(ChShurProduct& ShurProduct,
                                   ChProjectConstraints& Project,
                                   DynamicVector<real>& gamma,
                                   const DynamicVector<real>& r,
                                   DynamicVector<real>& tmp) {
    real gdiff = 1.0 / pow(data_manager->num_constraints, 2.0);
    ShurProduct(gamma, tmp);
    tmp = tmp - r;
    tmp = gamma - gdiff * (tmp);
    Project(tmp.data());
    tmp = (1.0 / gdiff) * (gamma - tmp);

    return Sqrt((double)(tmp, tmp));
}

uint ChSolverParallelAPGDREF::Solve(ChShurProduct& ShurProduct,
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

    bool verbose = false;
    bool useWarmStarting = true;
    if (verbose)
        std::cout << "Number of constraints: " << size << "\nNumber of variables  : " << data_manager->num_rigid_bodies
                  << std::endl;

    real L, t;
    real theta;
    real thetaNew;
    real Beta;
    real obj1, obj2;

    gamma_hat.resize(size);
    gammaNew.resize(size);
    g.resize(size);
    y.resize(size);
    yNew.resize(size);
    tmp.resize(size);

    residual = 10e30;

    // (1) gamma_0 = zeros(nc,1)
    if (!useWarmStarting)
        gamma = 0.0;

    // (2) gamma_hat_0 = ones(nc,1)
    gamma_hat = 1.0;

    // (3) y_0 = gamma_0
    y = gamma;

    // (4) theta_0 = 1
    theta = 1.0;

    thetaNew = theta;
    Beta = 0.0;
    obj1 = 0.0, obj2 = 0.0;

    // (5) L_k = norm(N * (gamma_0 - gamma_hat_0)) / norm(gamma_0 - gamma_hat_0)
    tmp = gamma - gamma_hat;
    L = Sqrt((double)(tmp, tmp));
    if (L > 0) {
        ShurProduct(tmp, tmp);
        L = Sqrt((double)(tmp, tmp)) / L;
    } else {
        L = 1;
    }

    // (6) t_k = 1 / L_k
    if (L > 0) {
        t = 1.0 / L;
    } else {
        L = 1;
        t = 1;
    }

    // (7) for k := 0 to N_max
    for (current_iteration = 0; current_iteration < (signed)max_iter; current_iteration++) {
        // (8) g = N * y_k - r
        ShurProduct(y, g);
        g = g - r;

        // (9) gamma_(k+1) = ProjectionOperator(y_k - t_k * g)
        gammaNew = y - t * g;
        Project(gammaNew.data());

        // (10) while 0.5 * gamma_(k+1)' * N * gamma_(k+1) - gamma_(k+1)' * r >= 0.5 * y_k' * N * y_k - y_k' * r + g' *
        // (gamma_(k+1) - y_k) + 0.5 * L_k * norm(gamma_(k+1) - y_k)^2
        ShurProduct(gammaNew, tmp);  // Here tmp is equal to N*gammaNew;
        obj1 = 0.5 * (gammaNew, tmp) - (gammaNew, r);
        ShurProduct(y, tmp);  // Here tmp is equal to N*y;
        obj2 = 0.5 * (y, tmp) - (y, r);
        tmp = gammaNew - y;  // Here tmp is equal to gammaNew - y
        obj2 = obj2 + (g, tmp) + 0.5 * L * (tmp, tmp);

        while (obj1 >= obj2) {
            // (11) L_k = 2 * L_k
            L = 2.0 * L;

            // (12) t_k = 1 / L_k
            t = 1.0 / L;

            // (13) gamma_(k+1) = ProjectionOperator(y_k - t_k * g)
            gammaNew = y - t * g;
            Project(gammaNew.data());

            // Update the components of the while condition
            ShurProduct(gammaNew, tmp);  // Here tmp is equal to N*gammaNew;
            obj1 = 0.5 * (gammaNew, tmp) - (gammaNew, r);
            ShurProduct(y, tmp);  // Here tmp is equal to N*y;
            obj2 = 0.5 * (y, tmp) - (y, r);
            tmp = gammaNew - y;  // Here tmp is equal to gammaNew - y
            obj2 = obj2 + (g, tmp) + 0.5 * L * (tmp, tmp);

            // (14) endwhile
        }

        // (15) theta_(k+1) = (-theta_k^2 + theta_k * Sqrt(theta_k^2 + 4)) / 2;
        thetaNew = (-pow(theta, 2.0) + theta * Sqrt(pow(theta, 2.0) + 4.0)) / 2.0;

        // (16) Beta_(k+1) = theta_k * (1 - theta_k) / (theta_k^2 + theta_(k+1))
        Beta = theta * (1.0 - theta) / (pow(theta, 2) + thetaNew);

        // (17) y_(k+1) = gamma_(k+1) + Beta_(k+1) * (gamma_(k+1) - gamma_k)
        yNew = gammaNew + Beta * (gammaNew - gamma);

        // (18) r = r(gamma_(k+1))
        real res = Res4(ShurProduct, Project, gammaNew, r, tmp);

        // (19) if r < epsilon_min
        if (res < residual) {
            // (20) r_min = r
            residual = res;

            // (21) gamma_hat = gamma_(k+1)
            gamma_hat = gammaNew;

            // (22) endif
        }

        // (23) if r < Tau
        if (verbose)
            std::cout << "Residual: " << residual << ", Iter: " << current_iteration << std::endl;

        DynamicVector<real> Nl(gammaNew.size());
        ShurProduct(gammaNew, Nl);         // 1)  g_tmp = N*l_candidate
        Nl = 0.5 * Nl - r;                 // 2) 0.5*N*l_candidate-b_shur
        objective_value = (gammaNew, Nl);  // 3)  mf_p  = l_candidate'*(0.5*N*l_candidate-b_shur)

        AtIterationEnd(residual, objective_value);
        if (residual < data_manager->settings.solver.tol_speed) {
            // (24) break
            break;

            // (25) endif
        }

        // (26) if g' * (gamma_(k+1) - gamma_k) > 0
        if ((g, gammaNew - gamma) > 0) {
            // (27) y_(k+1) = gamma_(k+1)
            yNew = gammaNew;

            // (28) theta_(k+1) = 1
            thetaNew = 1.0;

            // (29) endif
        }

        // (30) L_k = 0.9 * L_k
        L = 0.9 * L;

        // (31) t_k = 1 / L_k
        t = 1.0 / L;

        // Update iterates
        theta = thetaNew;
        gamma = gammaNew;
        y = yNew;

        // (32) endfor
    }

    // (33) return Value at time step t_(l+1), gamma_(l+1) := gamma_hat
    gamma = gamma_hat;

    return current_iteration;
}
