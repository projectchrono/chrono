#include "chrono_parallel/solver/ChSolverBB.h"
#include <blaze/math/CompressedVector.h>
using namespace chrono;

ChSolverBB::ChSolverBB() : ChSolverParallel() {}

void ChSolverBB::UpdateR() {
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

uint ChSolverBB::SolveBB(const uint max_iter,
                         const uint size,
                         const DynamicVector<real>& r,
                         DynamicVector<real>& gamma) {
    real& lastgoodres = data_manager->measures.solver.residual;
    //ChTimer<> t1, t2, t3, t4;
    //t1.start();

    real& residual = data_manager->measures.solver.residual;
    real& objective_value = data_manager->measures.solver.objective_value;
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
    double a_min = 1e-13;
    double a_max = 1e13;
    double sigma_min = 0.1;
    double sigma_max = 0.9;
    double alpha = 0.0001;
    double gmma = 1e-4;
    double gdiff = 0.000001;
    bool do_preconditioning = false;
    double neg_BB1_fallback = 0.11;
    double neg_BB2_fallback = 0.12;
    // Disable warm start for testing
    // ml = gamma;
    lastgoodres = 10e30;
    double lastgoodfval = 10e30;
    ml_candidate = ml;
    ShurProduct(ml, temp);
    mg = temp - r;
    mg_p = mg;

    real mf_p = 0;
    real mf = 1e29;
    int n_armijo = 10;
    int max_armijo_backtrace = 3;
    std::vector<real> f_hist;
    //t1.stop();

    for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
        //t2.start();
        temp = (ml - alpha * mg);
        Project(temp.data());
        mdir = temp - ml;

        real dTg = (mdir, mg);
        real lambda = 1.0;
        int n_backtracks = 0;
        bool armijo_repeat = true;
        //t2.stop();
        //t3.start();
        while (armijo_repeat) {
            ml_p = ml + lambda * mdir;

            ShurProduct(ml_p, temp);
            mg_p = temp - r;
            mf_p = (ml_p, 0.5 * temp - r);

            f_hist.push_back(mf_p);

            double max_compare = 10e29;
            for (int h = 1; h <= Min(current_iteration, n_armijo); h++) {
                double compare = f_hist[current_iteration - h] + gmma * lambda * dTg;
                if (compare > max_compare)
                    max_compare = compare;
            }
            if (mf_p > max_compare) {
                armijo_repeat = true;
                if (current_iteration > 0)
                    mf = f_hist[current_iteration - 1];
                real lambdanew = -lambda * lambda * dTg / (2 * (mf_p - mf - lambda * dTg));
                lambda = Max(sigma_min * lambda, Min(sigma_max * lambda, lambdanew));
                std::cout << " Repeat Armijo, new lambda=" << lambda << "\n";
            } else {
                armijo_repeat = false;
            }
            n_backtracks = n_backtracks + 1;
            if (n_backtracks > max_armijo_backtrace)
                armijo_repeat = false;
        }
        //t3.stop();
        //t4.start();
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
            ml_candidate = ml;
        }

        AtIterationEnd(lastgoodres, 0);

        //t4.stop();
    }
    //printf("TIME: [%f %f %f %f]\n", t1(), t2(), t3(), t4());

    gamma = ml_candidate;
    data_manager->system_timer.stop("ChSolverParallel_Solve");
    return current_iteration;
}
