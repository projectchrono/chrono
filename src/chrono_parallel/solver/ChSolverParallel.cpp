#include "chrono_parallel/solver/ChSolverParallel.h"

using namespace chrono;

ChSolverParallel::ChSolverParallel() {
    max_iteration = 100;
    current_iteration = 0;
    rigid_rigid = NULL;
    three_dof = NULL;
    fem = NULL;
    bilateral = NULL;
}

void ChSolverParallel::Project(real* gamma) {
    data_manager->system_timer.start("ChSolverParallel_Project");
    rigid_rigid->Project(gamma);

    three_dof->Project(gamma);
    fem->Project(gamma);

    data_manager->system_timer.stop("ChSolverParallel_Project");
}

void ChSolverParallel::Project_Single(int index, real* gamma) {
    data_manager->system_timer.start("ChSolverParallel_Project");
    rigid_rigid->Project_Single(index, gamma);
    data_manager->system_timer.stop("ChSolverParallel_Project");
}
//=================================================================================================================================

void ChSolverParallel::ComputeSRhs(custom_vector<real>& gamma,
                                   const custom_vector<real>& rhs,
                                   custom_vector<real3>& vel_data,
                                   custom_vector<real3>& omg_data,
                                   custom_vector<real>& b) {
    // TODO change SHRS to use blaze
    // ComputeImpulses(gamma, vel_data, omg_data);
    // rigid_rigid->ComputeS(rhs, vel_data, omg_data, b);
}

void ChSolverParallel::ShurProduct(const DynamicVector<real>& x, DynamicVector<real>& output) {
    data_manager->system_timer.start("ShurProduct");

    const DynamicVector<real>& E = data_manager->host_data.E;

    uint num_rigid_contacts = data_manager->num_rigid_contacts;
    uint num_rigid_fluid_contacts = data_manager->num_rigid_contacts;
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;
    output.reset();

    const CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    const CompressedMatrix<real>& M_invD = data_manager->host_data.M_invD;
    const CompressedMatrix<real>& Nshur = data_manager->host_data.Nshur;

    if (data_manager->settings.solver.local_solver_mode == data_manager->settings.solver.solver_mode) {
        if (data_manager->settings.solver.compute_N) {
            output = Nshur * x + E * x;
        } else {
            output = D_T * M_invD * x + E * x;
        }

    } else {
        const SubMatrixType& D_n_T = _DNT_;
        const SubMatrixType& D_b_T = _DBT_;
        const SubMatrixType& M_invD_n = _MINVDN_;
        const SubMatrixType& M_invD_b = _MINVDB_;

        SubVectorType o_b = subvector(output, num_unilaterals, num_bilaterals);
        ConstSubVectorType x_b = subvector(x, num_unilaterals, num_bilaterals);
        ConstSubVectorType E_b = subvector(E, num_unilaterals, num_bilaterals);

        SubVectorType o_n = subvector(output, 0, num_rigid_contacts);
        ConstSubVectorType x_n = subvector(x, 0, num_rigid_contacts);
        ConstSubVectorType E_n = subvector(E, 0, num_rigid_contacts);

        switch (data_manager->settings.solver.local_solver_mode) {
            case BILATERAL: {
                o_b = D_b_T * (M_invD_b * x_b) + E_b * x_b;
            } break;

            case NORMAL: {
                blaze::DynamicVector<real> tmp = M_invD_b * x_b + M_invD_n * x_n;
                o_b = D_b_T * tmp + E_b * x_b;
                o_n = D_n_T * tmp + E_n * x_n;
            } break;

            case SLIDING: {
                const SubMatrixType& D_t_T = _DTT_;
                const SubMatrixType& M_invD_t = _MINVDT_;
                SubVectorType o_t = subvector(output, num_rigid_contacts, num_rigid_contacts * 2);
                ConstSubVectorType x_t = subvector(x, num_rigid_contacts, num_rigid_contacts * 2);
                ConstSubVectorType E_t = subvector(E, num_rigid_contacts, num_rigid_contacts * 2);

                blaze::DynamicVector<real> tmp = M_invD_b * x_b + M_invD_n * x_n + M_invD_t * x_t;
                o_b = D_b_T * tmp + E_b * x_b;
                o_n = D_n_T * tmp + E_n * x_n;
                o_t = D_t_T * tmp + E_t * x_t;

            } break;

            case SPINNING: {
                const SubMatrixType& D_t_T = _DTT_;
                const SubMatrixType& D_s_T = _DST_;
                const SubMatrixType& M_invD_t = _MINVDT_;
                const SubMatrixType& M_invD_s = _MINVDS_;
                SubVectorType o_t = subvector(output, num_rigid_contacts, num_rigid_contacts * 2);
                ConstSubVectorType x_t = subvector(x, num_rigid_contacts, num_rigid_contacts * 2);
                ConstSubVectorType E_t = subvector(E, num_rigid_contacts, num_rigid_contacts * 2);

                SubVectorType o_s = subvector(output, num_rigid_contacts * 3, num_rigid_contacts * 3);
                ConstSubVectorType x_s = subvector(x, num_rigid_contacts * 3, num_rigid_contacts * 3);
                ConstSubVectorType E_s = subvector(E, num_rigid_contacts * 3, num_rigid_contacts * 3);

                blaze::DynamicVector<real> tmp = M_invD_b * x_b + M_invD_n * x_n + M_invD_t * x_t + M_invD_s * x_s;
                o_b = D_b_T * tmp + E_b * x_b;
                o_n = D_n_T * tmp + E_n * x_n;
                o_t = D_t_T * tmp + E_t * x_t;
                o_s = D_s_T * tmp + E_s * x_s;

            } break;
        }
    }
    data_manager->system_timer.stop("ShurProduct");
}

void ChSolverParallel::ShurBilaterals(const DynamicVector<real>& x, DynamicVector<real>& output) {
    const SubMatrixType& D_b_T = _DBT_;
    const SubMatrixType& M_invD_b = _MINVDB_;

    output = D_b_T * (M_invD_b * x);
}

//=================================================================================================================================

void ChSolverParallel::UpdatePosition(custom_vector<real>& x) {
    //
    //   if (rigid_rigid->solve_sliding == true || rigid_rigid->solve_spinning ==
    //   true) {
    //      return;
    //   }
    //   shurA(x.data());
    //
    //   data_manager->host_data.vel_new_data =
    //   data_manager->host_data.vel_data + data_manager->host_data.QXYZ_data;
    //   data_manager->host_data.omg_new_data +
    //   data_manager->host_data.omg_data + data_manager->host_data.QUVW_data;
    //
    //#pragma omp parallel for
    //   for (int i = 0; i < data_manager->num_bodies; i++) {
    //
    //      data_manager->host_data.pos_new_data[i] =
    //      data_manager->host_data.pos_data[i] +
    //      data_manager->host_data.vel_new_data[i] * step_size;
    //      //real3 dp =
    //      data_manager->host_data.pos_new_data[i]-data_manager->host_data.pos_data[i];
    //      //cout<<dp<<endl;
    //      real4 moldrot = data_manager->host_data.rot_data[i];
    //      real3 newwel = data_manager->host_data.omg_new_data[i];
    //
    //      M33 A = AMat(moldrot);
    //      real3 newwel_abs = MatMult(A, newwel);
    //      real mangle = length(newwel_abs) * step_size;
    //      newwel_abs = normalize(newwel_abs);
    //      real4 mdeltarot = Q_from_AngAxis(mangle, newwel_abs);
    //      real4 mnewrot = mdeltarot % moldrot;
    //      data_manager->host_data.rot_new_data[i] = mnewrot;
    //   }
}

void ChSolverParallel::UpdateContacts() {
    ////TODO: Re-implement this using new dispatch
    //   if (rigid_rigid->solve_sliding == true || rigid_rigid->solve_spinning ==
    //   true) {
    //      return;
    //   }
    //
    //   //// TODO:  This ASSUMES that we are using an MPR narrowphase!!
    //   ////        Instead of constructing a narrowphaseMPR object here,
    //   ////        modify so that we can use the CHCNarrowphase object
    //   ////        from the CollisionSystemParallel.
    //   collision::ChCNarrowphaseMPR narrowphase;
    //   narrowphase.SetCollisionEnvelope(data_manager->settings.collision.collision_envelope);
    //   narrowphase.Update(data_manager);
    //
    //   rigid_rigid->UpdateJacobians();
    //   rigid_rigid->UpdateRHS();
}

uint ChSolverParallel::SolveStab(const uint max_iter, const uint size, const ConstSubVectorType& mb, SubVectorType& x) {
    LOG(INFO) << "ChSolverParallel::SolveStab";
    real& residual = data_manager->measures.solver.residual;

    uint N = mb.size();

    v.resize(N);
    v_hat.resize(x.size());
    w.resize(N);
    Av.resize(x.size());
    const SubMatrixType& D_b_T = _DBT_;
    const SubMatrixType& M_invD_b = _MINVDB_;

    CompressedMatrix<real> NshurB = D_b_T * M_invD_b;

    real beta, c = 1, eta, norm_rMR, norm_r0, c_old = 1, s_old = 0, s = 0, alpha, beta_old, c_oold, s_oold, r1_hat, r1,
               r2, r3;
    v_hat = NshurB * x;  // ShurBilaterals(x, v_hat);
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

    for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
        //// Lanczos
        v_old = v;
        v = 1.0 / beta * v_hat;
        Av = NshurB * v;  // ShurBilaterals(v, Av);
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
    LOG(INFO) << "Done ChSolverParallel::SolveStab";
    return current_iteration;
}
