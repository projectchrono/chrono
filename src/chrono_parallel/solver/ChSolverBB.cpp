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

    bool do_BB1e2 = true;
    bool do_BB1 = false;
    bool do_BB2 = false;
    double neg_BB1_fallback = 0.11;
    double neg_BB2_fallback = 0.12;
    // Disable warm start for testing
    // ml = gamma;
    double lastgoodres = 10e30;
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

    for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
        temp = (ml - alpha * mg);
        Project(temp.data());
        mdir = temp - ml;
//        for (int i = 0; i < mdir.size() / 3; i++) {
//            printf("mdir %.20f\n", mdir[i]);
//        }
        real dTg = (mdir, mg);
        //printf("dTg: %.20f\n", dTg);
        real lambda = 1.0;
        int n_backtracks = 0;
        bool armijo_repeat = true;
        while (armijo_repeat) {
            ml_p = ml + lambda * mdir;
//            for (int i = 0; i < ml_p.size() / 3; i++) {
//                printf("ml_p %.20f\n", ml_p[i]);
//            }
            ShurProduct(ml_p, temp);
            mg_p = temp - r;
            mf_p = (ml_p, 0.5 * temp - r);
            //printf("mf_p: %.20f\n", mf_p);

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

        ms = ml_p - ml;
        my = mg_p - mg;
        ml = ml_p;
        mg = mg_p;

        if (((do_BB1e2) && (current_iteration % 2 == 0)) || do_BB1) {
            temp = ms;
            real sDs = (ms, temp);
            real sy = (ms, my);
            if (sy <= 0) {
                alpha = neg_BB1_fallback;
            } else {
                real alph = sDs / sy;  // (s,Ds)/(s,y)   BB1
                alpha = Min(a_max, Max(a_min, alph));
            }
        }
        if (((do_BB1e2) && (current_iteration % 2 != 0)) || do_BB2) {
            double sy = (ms, my);
            temp = my;
            double yDy = (my, temp);
            if (sy <= 0) {
                alpha = neg_BB2_fallback;
            } else {
                double alph = sy / yDy;  // (s,y)/(y,Di*y)   BB2
                alpha = Min(a_max, Max(a_min, alph));
            }
        }
        temp = ml - gdiff * mg;
        Project(temp.data());
        temp = (ml - temp) / (-gdiff);

        real g_proj_norm = Sqrt((temp, temp));
        //printf("g_proj_norm: %.20f\n", g_proj_norm);
        if (g_proj_norm < lastgoodres) {
            lastgoodres = g_proj_norm;
            ml_candidate = ml;
        }
//        for (int i = 0; i < ml.size() / 3; i++) {
//            printf("ml %.20f\n", ml[i]);
//        }
    }
    gamma = ml_candidate;
    data_manager->system_timer.stop("ChSolverParallel_Solve");
    return current_iteration;
}
