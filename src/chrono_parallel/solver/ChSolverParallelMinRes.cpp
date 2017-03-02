#include "chrono_parallel/solver/ChSolverParallel.h"

using namespace chrono;

uint ChSolverParallelMinRes::Solve(ChShurProduct& ShurProduct,
                                   ChProjectConstraints& Project,
                                   const uint max_iter,
                                   const uint size,
                                   const DynamicVector<real>& mb,
                                   DynamicVector<real>& x) {
    if (size == 0) {
        return 0;
    }

    real& residual = data_manager->measures.solver.residual;
    real& objective_value = data_manager->measures.solver.objective_value;

    uint N = (uint)mb.size();

    v.resize(N);
    v_hat.resize(x.size());
    w.resize(N);
    Av.resize(x.size());

    real beta, c = 1, eta, norm_rMR, norm_r0, c_old = 1, s_old = 0, s = 0, alpha, beta_old, c_oold, s_oold, r1_hat, r1,
               r2, r3;
    ShurProduct(x, v_hat);
    v_hat = mb - v_hat;
    beta = Sqrt((v_hat, v_hat));
    w_old = w;
    eta = beta;
    xMR = x;
    norm_rMR = beta;
    norm_r0 = beta;
    v = 0;
    w = 0;

    if (beta == 0 || norm_rMR / norm_r0 < data_manager->settings.solver.tol_speed) {
        return 0;
    }

    for (current_iteration = 0; current_iteration < (signed)max_iter; current_iteration++) {
        //// Lanczos
        v_old = v;
        v = 1.0 / beta * v_hat;
        ShurProduct(v, Av);
        alpha = (v, Av);
        v_hat = Av - alpha * v - beta * v_old;
        beta_old = beta;
        beta = Sqrt((v_hat, v_hat));
        //// QR factorization
        c_oold = c_old;
        c_old = c;
        s_oold = s_old;
        s_old = s;
        r1_hat = c_old * alpha - c_oold * s_old * beta_old;
        r1 = 1 / Sqrt(r1_hat * r1_hat + beta * beta);
        r2 = s_old * alpha + c_oold * c_old * beta_old;
        r3 = s_oold * beta_old;
        //// Givens Rotation
        c = r1_hat * r1;
        s = beta * r1;
        //// update
        w_oold = w_old;
        w_old = w;
        w = r1 * (v - r3 * w_oold - r2 * w_old);
        x = x + c * eta * w;
        norm_rMR = norm_rMR * Abs(s);
        eta = -s * eta;
        residual = norm_rMR / norm_r0;

        real maxdeltalambda = 0;  // CompRes(mb, num_contacts);      //NormInf(ms);
        AtIterationEnd(residual, maxdeltalambda);

        if (residual < data_manager->settings.solver.tol_speed) {
            break;
        }
    }

    return current_iteration;
}
