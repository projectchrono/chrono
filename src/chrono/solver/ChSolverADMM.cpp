// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#include <cmath>

#include "chrono/core/ChSparsityPatternLearner.h"
#include "chrono/utils/ChConstants.h"

#include "chrono/solver/ChSolverADMM.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverADMM)

ChSolverADMM::ChSolverADMM()
    : r_prim(0),
      r_dual(0),
      precond(false),
      rho(0.1),
      rho_b(1e-9),
      sigma(1e-6),
      stepadjust_each(5),
      stepadjust_threshold(1.5),
      stepadjust_maxfactor(50),
      stepadjust_type(AdmmStepType::BALANCED_FAST),
      tol_prim(1e-6),
      tol_dual(1e-6),
      acceleration(AdmmAcceleration::BASIC) {
    LS_solver = chrono_types::make_shared<ChSolverSparseQR>();
}

ChSolverADMM::ChSolverADMM(std::shared_ptr<ChDirectSolverLS> my_LS_engine) : ChSolverADMM() {
    this->LS_solver = my_LS_engine;
}

double ChSolverADMM::Solve(ChSystemDescriptor& sysd) {
    switch (this->acceleration) {
        case AdmmAcceleration::BASIC:
            return _SolveBasic(sysd);
        case AdmmAcceleration::NESTEROV:
            return _SolveFast(sysd);
        default:
            return _SolveBasic(sysd);
    }
}

/// Performs basic ADMM, as in solve_kkt_ADMMbasic.m prototype

double ChSolverADMM::_SolveBasic(ChSystemDescriptor& sysd) {
    ChTimer m_timer_convert;
    ChTimer m_timer_factorize;
    ChTimer m_timer_solve;

    double rho_i = this->rho;

    std::vector<ChConstraint*>& mconstraints = sysd.GetConstraints();

    int nc = sysd.CountActiveConstraints();
    int nv = sysd.CountActiveVariables();

    ChVectorDynamic<> v(nv);

    // shortcut for the case of no constraints:
    if (nc == 0) {
        ChSparseMatrix H(nv, nv);
        ChVectorDynamic<> k(nv);

        m_timer_convert.start();

        // sysd.BuildSystemMatrix(0, &LS_solver->A(), 0, &LS_solver->b(), 0, 0, 0);
        // sysd.BuildSystemMatrix(0, &LS_solver->A(), 0, &LS_solver->b(), 0, 0, 0);
        // much faster to fill brand new sparse matrices??!!

        ChSparsityPatternLearner sparsity_pattern(nv, nv);
        sysd.BuildSystemMatrix(&sparsity_pattern, nullptr);
        sparsity_pattern.Apply(H);
        sysd.BuildSystemMatrix(&H, &k);
        LS_solver->A() = H;
        LS_solver->b() = k;

        m_timer_convert.stop();
        if (verbose)
            std::cout << " Time for BuildSystemMatrix: << " << m_timer_convert.GetTimeSeconds() << "s" << std::endl;

        // v = H\k
        LS_solver->SetupCurrent();
        LS_solver->SolveCurrent();

        // v = LS_solver->x();
        sysd.FromVectorToVariables(LS_solver->x());

        return 0;
    }

    ChSparseMatrix Cq(nc, nv);
    Cq.setZeroValues();
    ChSparseMatrix E(nc, nc);
    E.setZeroValues();
    ChVectorDynamic<> k(nv);
    k.setZero(nv);
    ChVectorDynamic<> b(nc);
    b.setZero(nc);

    ChSparseMatrix A(nv + nc, nv + nc);
    ChVectorDynamic<> B(nv + nc);
    // ChVectorDynamic<> X(nv+nc);

    ChVectorDynamic<> l(nc);
    ChVectorDynamic<> z(nc);
    ChVectorDynamic<> y(nc);

    ChVectorDynamic<> l_old(nc);
    ChVectorDynamic<> z_old(nc);
    ChVectorDynamic<> y_old(nc);

    sysd.PasteConstraintsJacobianMatrixInto(Cq);
    sysd.PasteComplianceMatrixInto(E);
    sysd.BuildFbVector(k);
    sysd.BuildBiVector(b);
    Cq.makeCompressed();
    E.makeCompressed();

    if (!this->m_warm_start) {
        l.setZero();
        z.setZero();
        y.setZero();
    } else {
        // warmstarted l:
        sysd.FromConstraintsToVector(l, false);

        // warmstarted v:
        // v = H\(k + D*l); // PERFORMANCE HIT, probably better reuse last v if possible..
        sysd.FromVariablesToVector(
            v, false);  // this works supposing that variables have been warmstarted with "v" too, otherwise:

        // warmstarted y:
        // the following correct only if r_dual was approx.=0. //// TODO  as parameter
        y = -(Cq * v - E * l + b);  //  dual residual exploiting the kkt form instead of - (N*l+r), faster!

        /*
        std::cout << "Y warmastarted:" << std::endl;
        for (int k = 0; k < std::min(y.rows(), 10); ++k)
            std::cout << "  " << y(k) << std::endl;
        */

        // warmstarted z:
        z = l;  //  warm start also this - should project? //// TODO  as parameter
        // sysd.ConstraintsProject(z);

        // y_hat  = y;   // only for spectral stepsize
    }

    ChVectorDynamic<> S(nc);

    if (this->precond == true) {
        // Compute diagonal values of N , only mass effect, neglecting stiffness for the moment, TODO
        //  g_i=[Cq_i]*[invM_i]*[Cq_i]'
        for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
            mconstraints[ic]->UpdateAuxiliary();

        // Average all g_i for the triplet of contact constraints n,u,v.
        int j_friction_comp = 0;
        double gi_values[3];
        for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
            if (mconstraints[ic]->GetMode() == ChConstraint::Mode::FRICTION) {
                gi_values[j_friction_comp] = mconstraints[ic]->GetSchurComplement();
                j_friction_comp++;
                if (j_friction_comp == 3) {
                    double average_g_i = (gi_values[0] + gi_values[1] + gi_values[2]) * CH_1_3;
                    mconstraints[ic - 2]->SetSchurComplement(average_g_i);
                    mconstraints[ic - 1]->SetSchurComplement(average_g_i);
                    mconstraints[ic - 0]->SetSchurComplement(average_g_i);
                    j_friction_comp = 0;
                }
            }
        }
        // The vector with the diagonal of the N matrix
        S.setZero();
        int d_i = 0;
        for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
            if (mconstraints[ic]->IsActive()) {
                S(d_i, 0) = std::sqrt(
                    mconstraints[ic]->GetSchurComplement());  // square root of diagonal of N, just mass matrices
                                                              // considered, no stiffness matrices anyway
                ++d_i;
            }
        // Now we should scale Cq, E, b as
        // Cq = Cq*diag(S);
        // E = diag(S)*E*diag(S);

        // but to avoid storing Cq and E and assembly in A, we postpone this by scaling the entire A matrix later via a
        // IS*A*IS operation

        // b = diag(S)*b;
        b = b.cwiseProduct(S);

        // warm started values must be scaled too
        l = l.cwiseQuotient(S);  // from l to \breve{l}
        z = z.cwiseQuotient(S);
        y = y.cwiseProduct(S);

    } else {
        S.setConstant(1);
    }

    // vsigma = ones(nconstr,1)*sigma;
    ChVectorDynamic<> vsigma(nc);  // not needed
    vsigma.setConstant(this->sigma);

    // vrho = ones(nconstr,1)*rho;
    ChVectorDynamic<> vrho(nc);
    vrho.setConstant(rho_i);

    // vrho(fric==-2) = rho_b;          // special step for bilateral joints
    int s_c = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
        if (mconstraints[ic]->IsActive()) {
            if (mconstraints[ic]->GetMode() == ChConstraint::Mode::LOCK)
                vrho(s_c) = rho_b;
            s_c++;
        }
    }

    // FACTORIZATION
    //
    // A = [M, Cq'; Cq, -diag(vsigma+vrho) + E ];

    m_timer_convert.start();

    LS_solver->A().resize(nv + nc, nv + nc);  // otherwise conservativeResize in BuildSystemMatrix() causes error

    // sysd.BuildSystemMatrix(&LS_solver->A(),&LS_solver->b());  // A = [M, Cq'; Cq, E ];
    // much faster to fill brand new sparse matrices??!!
    ChSparsityPatternLearner sparsity_pattern(nv + nc, nv + nc);
    sysd.BuildSystemMatrix(&sparsity_pattern, nullptr);
    sparsity_pattern.Apply(A);

    sysd.BuildSystemMatrix(&A, &B);  // A = [M, Cq'; Cq, E ];

    if (this->precond) {
        // the following is equivalent to having scaled
        // Cq = Cq*diag(S);
        // E = diag(S)*E*diag(S);
        ChVectorDynamic<> IS(nv + nc);
        IS << Eigen::VectorXd::Ones(nv), S;
        A = IS.asDiagonal() * A * IS.asDiagonal();
        B = IS.asDiagonal() * B;  // not needed? B here only factorization...
    }

    LS_solver->A() = A;
    LS_solver->b() = B;

    for (int i = 0; i < nc; ++i)
        LS_solver->A().coeffRef(nv + i, nv + i) += -(sigma + vrho(i));  //  A = [M, Cq'; Cq, -diag(vsigma+vrho) + E ];

    m_timer_convert.stop();
    if (verbose)
        std::cout << " Time for BuildSystemMatrix: << " << m_timer_convert.GetTimeSeconds() << "s" << std::endl;

    m_timer_factorize.start();

    LS_solver->SetupCurrent();  // LU decomposition ++++++++++++++++++++++++++++++++++++++

    m_timer_factorize.stop();
    if (verbose)
        std::cout << " Time for factorize : << " << m_timer_factorize.GetTimeSeconds() << "s" << std::endl;

    /*
    res_story.r_prim=zeros(1,1);
    res_story.r_dual_Nlry=zeros(1,1);
    res_story.r_dual=zeros(1,1);
    res_story.r_combined_pre=zeros(1,1);
    res_story.r_combined=zeros(1,1);
    res_story.r_deltal=zeros(1,1);
    res_story.r_rho=zeros(1,1);
    res_story.r_violation=zeros(1,1);
    */

    std::fill(violation_history.begin(), violation_history.end(), 0.0);
    std::fill(dlambda_history.begin(), dlambda_history.end(), 0.0);

    for (int iter = 0; iter < m_max_iterations; iter++) {
        // diagnostic
        l_old = l;
        z_old = z;
        y_old = y;

        // X   (lambda)

        // SOLVE LINEAR SYSTEM HERE
        // ckkt = -bkkt + (vsigma+vrho).*z - y;

        ChVectorDynamic<> ckkt = -b + (vsigma + vrho).cwiseProduct(z) - y;
        LS_solver->b() << k, ckkt;  // B = [k;ckkt];

        m_timer_solve.start();

        LS_solver->SolveCurrent();  // LU forward/backsolve ++++++++++++++++++++++++++++++++++++++

        m_timer_solve.stop();
        if (verbose)
            std::cout << " Time for solve : << " << m_timer_solve.GetTimeSeconds() << "s" << std::endl;

        // x = dA\B;      // A* x = B  with x = [v, -l]
        l = -LS_solver->x().block(nv, 0, nc, 1);
        v = LS_solver->x().block(0, 0, nv, 1);

        // Z

        //    z = project_orthogonal(l + y. / vrho, fric);
        z = l + y.cwiseQuotient(vrho);
        sysd.ConstraintsProject(z);

        // Y

        y = y + vrho.asDiagonal() * (l - z);

        // y_hat = y + vrho .* (l - z_old);

        // Compute residuals for tolerances

        r_prim = ((z - l).cwiseProduct(S)).lpNorm<Eigen::Infinity>();  // r_prim     = norm((z - l).*S, inf);
        double r_prim_pre = (z - l).lpNorm<Eigen::Infinity>();         // r_prim_pre = norm((z - l)   , inf);

        r_dual = (((z - z_old).cwiseProduct(vrho)).cwiseQuotient(S))
                     .lpNorm<Eigen::Infinity>();  // r_dual = norm((vrho.*(z - z_old)). / S, inf); % even faster!See
                                                  // book of Boyd.But coincides only for alpha = 1 !!!
        double r_dual_pre = ((z - z_old).cwiseProduct(vrho))
                                .lpNorm<Eigen::Infinity>();  // r_dual_pre     = norm((vrho.*(z - z_old))   ,inf);

        // r_combined = norm((z - l).*S, 2) + norm((z - z_old). / S, 2);% combined res.in original metric
        // r_combined_pre = norm((z - l), 2) + norm((z - z_old), 2);% combined res.in precond.metric

        /*
        res_story.r_prim(j) = r_prim;
        res_story.r_dual_Nlry(j) = r_dual_Nlry;
        res_story.r_dual(j) = r_dual;
        res_story.r_combined_pre(j) = r_combined_pre;
        res_story.r_combined(j) = r_combined;
        res_story.r_rho(j) = rho_i;
        res_story.r_deltal(j) = norm((l - l_old).*S, inf);% diagnostic
        */

        if (verbose)
            std::cout << "ADMM iter=" << iter << " prim=" << r_prim << " dual=" << r_dual << "  rho=" << rho_i
                      << "  tols=" << this->tol_prim << " " << this->tol_dual << "" << std::endl;

        // For recording into violation history, if debugging
        if (this->record_violation_history) {
            // combined residual
            double r_combined = ((z - l).cwiseProduct(S)).norm() + ((z - z_old).cwiseQuotient(S)).norm();
            AtIterationEnd(r_combined, (l - l_old).norm(), iter);
        }

        // Termination:
        if ((r_prim < this->tol_prim) && (r_dual < this->tol_dual)) {
            if (verbose)
                std::cout << "ADMM converged ok! at iter=" << iter << std::endl;
            break;
        }

        // once in a while update the rho step parameter
        if ((iter % this->stepadjust_each) == 0) {
            double rhofactor = 1;  // default do not shrink / enlarge

            if (this->stepadjust_type == AdmmStepType::NONE) {
                rhofactor = 1.;
            }

            if (this->stepadjust_type == AdmmStepType::BALANCED_UNSCALED) {
                rhofactor = std::sqrt(r_prim_pre / r_dual_pre);
            }

            if (this->stepadjust_type == AdmmStepType::BALANCED_FAST) {
                double r_prim_scaled =
                    r_prim_pre / (std::max(z.lpNorm<Eigen::Infinity>(), l.lpNorm<Eigen::Infinity>()) +
                                  1e-10);  // maybe norm(l, inf) very similar to norm(z, inf)
                double r_dual_scaled =
                    r_dual_pre /
                    (y.lpNorm<Eigen::Infinity>() +
                     1e-10);  //  as in "ADMM Penalty Parameter Selection by Residual Balancing", Brendt Wohlberg
                rhofactor = std::sqrt(r_prim_scaled / (r_dual_scaled + 1e-10));
            }

            if (this->stepadjust_type == AdmmStepType::BALANCED_RANGE) {
                double r_prim_scaled = r_prim / this->tol_prim;
                double r_dual_scaled = r_dual / this->tol_dual;
                rhofactor = std::sqrt(r_prim_scaled / (r_dual_scaled + 1e-10));
            }

            // safeguards against extreme shrinking
            if (rhofactor < 1.0 / this->stepadjust_maxfactor) {
                rhofactor = 1.0 / this->stepadjust_maxfactor;
            }
            if (rhofactor > this->stepadjust_maxfactor) {
                rhofactor = this->stepadjust_maxfactor;
            }

            if ((rhofactor > this->stepadjust_threshold) || (rhofactor < 1.0 / this->stepadjust_threshold)) {
                ChTimer m_timer_refactorize;
                m_timer_refactorize.start();

                // Avoid rebuilding all sparse matrix:
                // A) just remove old rho with -= :
                for (int i = 0; i < nc; ++i)
                    LS_solver->A().coeffRef(nv + i, nv + i) -= -(sigma + vrho(i));

                // Update rho
                rho_i = rho_i * rhofactor;

                // vrho(fric == -2) = rho_b; //  special step for bilateral joints
                vrho.setConstant(rho_i);
                s_c = 0;
                for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
                    if (mconstraints[ic]->IsActive()) {
                        if (mconstraints[ic]->GetMode() == ChConstraint::Mode::LOCK)
                            vrho(s_c) = rho_b;
                        s_c++;
                    }
                }

                // UPDATE FACTORIZATION
                //
                //  A = [M, Cq'; Cq, -diag(vsigma+vrho) + E ];
                //
                // To avoid rebuilding A, we just removed the rho step from the diagonal in A), and now:
                // B) add old rho with += :
                for (int i = 0; i < nc; ++i)
                    LS_solver->A().coeffRef(nv + i, nv + i) += -(sigma + vrho(i));

                LS_solver->SetupCurrent();  // LU decomposition ++++++++++++++++++++++++++++++++++++++

                m_timer_refactorize.stop();
                if (verbose)
                    std::cout << " Time for re-factorize : << " << m_timer_refactorize.GetTimeSeconds() << "s"
                              << std::endl;
            }

        }  // end step adjust

    }  // end iteration

    l = l.cwiseProduct(S);

    /*
    std::cout << "Y resulting:" << std::endl;
        for (int k = 0; k < std::min(y.rows(), 10); ++k)
            std::cout << "  " << y(k) << std::endl;
    */

    sysd.FromVectorToConstraints(l);
    sysd.FromVectorToVariables(v);

    return r_dual;
}

/// Performs ADMM with Nesterov acceleration, as in solve_kkt_ADMMfast.m prototype

double ChSolverADMM::_SolveFast(ChSystemDescriptor& sysd) {
    // For Nesterov:
    double nalpha = 1.0;
    double c_k = 1e10;
    double eta = 0.9999;

    ChTimer m_timer_convert;
    ChTimer m_timer_factorize;
    ChTimer m_timer_solve;

    double rho_i = this->rho;

    std::vector<ChConstraint*>& mconstraints = sysd.GetConstraints();

    int nc = sysd.CountActiveConstraints();
    int nv = sysd.CountActiveVariables();

    ChVectorDynamic<> v(nv);

    // shortcut for the case of no constraints:
    if (nc == 0) {
        ChSparseMatrix H(nv, nv);
        ChVectorDynamic<> k(nv);

        m_timer_convert.start();

        ChSparsityPatternLearner sparsity_pattern(nv, nv);
        sysd.BuildSystemMatrix(&sparsity_pattern, 0);
        sparsity_pattern.Apply(H);
        sysd.BuildSystemMatrix(&H, &k);
        LS_solver->A() = H;
        LS_solver->b() = k;

        m_timer_convert.stop();
        if (verbose)
            std::cout << " Time for BuildSystemMatrix: << " << m_timer_convert.GetTimeSeconds() << "s" << std::endl;

        // v = H\k
        LS_solver->SetupCurrent();
        LS_solver->SolveCurrent();

        // v = LS_solver->x();
        sysd.FromVectorToVariables(LS_solver->x());

        return 0;
    }

    ChSparseMatrix Cq(nc, nv);
    Cq.setZeroValues();
    ChSparseMatrix E(nc, nc);
    E.setZeroValues();
    ChVectorDynamic<> k(nv);
    k.setZero(nv);
    ChVectorDynamic<> b(nc);
    b.setZero(nc);

    ChSparseMatrix A(nv + nc, nv + nc);
    ChVectorDynamic<> B(nv + nc);

    ChVectorDynamic<> l(nc);
    ChVectorDynamic<> z(nc);
    ChVectorDynamic<> y(nc);

    ChVectorDynamic<> l_old(nc);
    ChVectorDynamic<> z_old(nc);
    ChVectorDynamic<> y_old(nc);
    ChVectorDynamic<> v_old(nv);

    sysd.PasteConstraintsJacobianMatrixInto(Cq);
    sysd.PasteComplianceMatrixInto(E);
    sysd.BuildFbVector(k);
    sysd.BuildBiVector(b);
    Cq.makeCompressed();
    E.makeCompressed();

    if (!this->m_warm_start) {
        l.setZero();
        z.setZero();
        y.setZero();

        ///// TODO or just accept approximation?
        // v = M\(k + D*l); % PERFORMANCE HIT, only needed if truncating Nesterov before convergence
        v.setZero();  // enough approx? or better sysd.FromVariablesToVector(v, false); ?
    } else {
        // warmstarted l:
        sysd.FromConstraintsToVector(l, false);

        // warmstarted v:
        // v = H\(k + D*l); // PERFORMANCE HIT, probably better reuse last v if possible..
        sysd.FromVariablesToVector(
            v, false);  // this works supposing that variables have been warmstarted with "v" too, otherwise:

        // warmstarted y:
        // the following correct only if r_dual was approx.=0. //// TODO  as parameter
        y = -(Cq * v - E * l + b);  //  dual residual exploiting the kkt form instead of - (N*l+r), faster!

        /*
        std::cout << "Y warmastarted:" << std::endl;
        for (int k = 0; k < std::min(y.rows(), 10); ++k)
            std::cout << "  " << y(k) << std::endl;
        */

        // warmstarted z:
        z = l;  //  warm start also this - should project? //// TODO  as parameter
        // sysd.ConstraintsProject(z);
    }

    ChVectorDynamic<> S(nc);

    if (this->precond == true) {
        // Compute diagonal values of N , only mass effect, neglecting stiffness for the moment, TODO
        //  g_i=[Cq_i]*[invM_i]*[Cq_i]'
        for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
            mconstraints[ic]->UpdateAuxiliary();

        // Average all g_i for the triplet of contact constraints n,u,v.
        int j_friction_comp = 0;
        double gi_values[3];
        for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
            if (mconstraints[ic]->GetMode() == ChConstraint::Mode::FRICTION) {
                gi_values[j_friction_comp] = mconstraints[ic]->GetSchurComplement();
                j_friction_comp++;
                if (j_friction_comp == 3) {
                    double average_g_i = (gi_values[0] + gi_values[1] + gi_values[2]) * CH_1_3;
                    mconstraints[ic - 2]->SetSchurComplement(average_g_i);
                    mconstraints[ic - 1]->SetSchurComplement(average_g_i);
                    mconstraints[ic - 0]->SetSchurComplement(average_g_i);
                    j_friction_comp = 0;
                }
            }
        }
        // The vector with the diagonal of the N matrix
        S.setZero();
        int d_i = 0;
        for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
            if (mconstraints[ic]->IsActive()) {
                S(d_i, 0) = std::sqrt(
                    mconstraints[ic]->GetSchurComplement());  // square root of diagonal of N, just mass matrices
                                                              // considered, no stiffness matrices anyway
                ++d_i;
            }
        // Now we should scale Cq, E, b as
        // Cq = Cq*diag(S);
        // E = diag(S)*E*diag(S);

        // but to avoid storing Cq and E and assembly in A, we postpone this by scaling the entire A matrix later via a
        // IS*A*IS operation

        // b = diag(S)*b;
        b = b.cwiseProduct(S);

        // warm started values must be scaled too
        l = l.cwiseQuotient(S);  // from l to \breve{l}
        z = z.cwiseQuotient(S);
        y = y.cwiseProduct(S);

    } else {
        S.setConstant(1);
    }

    // vsigma = ones(nconstr,1)*sigma;
    ChVectorDynamic<> vsigma(nc);  // not needed
    vsigma.setConstant(this->sigma);

    // vrho = ones(nconstr,1)*rho;
    ChVectorDynamic<> vrho(nc);
    vrho.setConstant(rho_i);

    // vrho(fric==-2) = rho_b;          // special step for bilateral joints
    int s_c = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
        if (mconstraints[ic]->IsActive()) {
            if (mconstraints[ic]->GetMode() == ChConstraint::Mode::LOCK)
                vrho(s_c) = rho_b;
            s_c++;
        }
    }

    // FACTORIZATION
    //
    // A = [M, Cq'; Cq, -diag(vsigma+vrho) + E ];

    m_timer_convert.start();

    LS_solver->A().resize(nv + nc, nv + nc);  // otherwise conservativeResize in BuildSystemMatrix() causes error

    // sysd.BuildSystemMatrix(&LS_solver->A(),&LS_solver->b());  // A = [M, Cq'; Cq, E ];
    // much faster to fill brand new sparse matrices??!!
    ChSparsityPatternLearner sparsity_pattern(nv + nc, nv + nc);
    sysd.BuildSystemMatrix(&sparsity_pattern, nullptr);
    sparsity_pattern.Apply(A);

    sysd.BuildSystemMatrix(&A, &B);  // A = [M, Cq'; Cq, E ];

    if (this->precond) {
        // the following is equivalent to having scaled
        // Cq = Cq*diag(S);
        // E = diag(S)*E*diag(S);
        ChVectorDynamic<> IS(nv + nc);
        IS << Eigen::VectorXd::Ones(nv), S;
        A = IS.asDiagonal() * A * IS.asDiagonal();
        B = IS.asDiagonal() * B;  // not needed? B here only factorization...
    }

    LS_solver->A() = A;
    LS_solver->b() = B;

    for (int i = 0; i < nc; ++i)
        LS_solver->A().coeffRef(nv + i, nv + i) += -(sigma + vrho(i));  //  A = [M, Cq'; Cq, -diag(vsigma+vrho) + E ];

    m_timer_convert.stop();
    if (verbose)
        std::cout << " Time for BuildSystemMatrix: << " << m_timer_convert.GetTimeSeconds() << "s" << std::endl;

    m_timer_factorize.start();

    LS_solver->SetupCurrent();  // LU decomposition ++++++++++++++++++++++++++++++++++++++

    m_timer_factorize.stop();
    if (verbose)
        std::cout << " Time for factorize : << " << m_timer_factorize.GetTimeSeconds() << "s" << std::endl;

    /*
    res_story.r_prim=zeros(1,1);
    res_story.r_dual_Nlry=zeros(1,1);
    res_story.r_dual=zeros(1,1);
    res_story.r_combined_pre=zeros(1,1);
    res_story.r_combined=zeros(1,1);
    res_story.r_deltal=zeros(1,1);
    res_story.r_rho=zeros(1,1);
    res_story.r_violation=zeros(1,1);
    */

    for (int iter = 0; iter < m_max_iterations; iter++) {
        // diagnostic
        l_old = l;
        z_old = z;
        y_old = y;
        v_old = v;

        // Z  u

        // z = project_orthogonal( l + y./vrho,  fric);
        z = l + y.cwiseQuotient(vrho);
        sysd.ConstraintsProject(z);

        // X   (lambda)

        // SOLVE LINEAR SYSTEM HERE
        // ckkt = -bkkt + (vsigma+vrho).*z - y;

        ChVectorDynamic<> ckkt = -b + (vsigma + vrho).cwiseProduct(z) - y;
        LS_solver->b() << k, ckkt;  // B = [k;ckkt];

        m_timer_solve.start();

        LS_solver->SolveCurrent();  // LU forward/backsolve ++++++++++++++++++++++++++++++++++++++

        m_timer_solve.stop();
        if (verbose)
            std::cout << " Time for solve : << " << m_timer_solve.GetTimeSeconds() << "s" << std::endl;

        // x = dA\B;      // A* x = B  with x = [v, -l]
        l = -LS_solver->x().block(nv, 0, nc, 1);
        v = LS_solver->x().block(0, 0, nv, 1);

        // Y

        y = y + vrho.asDiagonal() * (l - z);

        // Compute residuals for tolerances
        /*
        r_prim         = norm((z-l).*S, inf);
        r_prim_pre     = norm((z-l)   , inf);
        r_dual_Nlry    = norm((D'*v-E*l+bkkt+y)./S,inf); % dual residual exploiting the kkt form instead of
        (N*l+r+y)./S, faster! r_dual_Nlry_pre= norm((D'*v-E*l+bkkt+y)   ,inf); % note: does not coincide with r_dual as
        in plain ADMM, discover why r_dual         = norm((vrho.*(l-l_old))./S,inf); % for order is Z-X-Y , different
        than in plain ADMM X-Z-Y - BUT NOT AFTER NESTEROV CORRECTION! r_dual_pre     = norm((vrho.*(l-l_old))   ,inf);
        r_combined     = norm((z-l).*S, 2) +  norm((l-l_old).*S, 2); % combined res. in original metric
        r_combined_pre = norm((z-l)   , 2) +  norm((l-l_old)    ,2); % combined res. in precond.metric
        */
        r_prim = ((z - l).cwiseProduct(S)).lpNorm<Eigen::Infinity>();  // r_prim     = norm((z - l).*S, inf);
        double r_prim_pre = (z - l).lpNorm<Eigen::Infinity>();         // r_prim_pre = norm((z - l)   , inf);

        r_dual = (((l - l_old).cwiseProduct(vrho)).cwiseQuotient(S))
                     .lpNorm<Eigen::Infinity>();  // r_dual = norm((vrho.*(l - l_old)). / S, inf); % even faster!See
                                                  // book of Boyd.But coincides only for alpha = 1 !!!
        double r_dual_pre = ((l - l_old).cwiseProduct(vrho))
                                .lpNorm<Eigen::Infinity>();  // r_dual_pre     = norm((vrho.*(l - l_old))   ,inf);

        // r_combined = norm((z - l).*S, 2) + norm((l - l_old). / S, 2);% combined res.in original metric
        // r_combined_pre = norm((z - l), 2) + norm((l - l_old), 2);% combined res.in precond.metric

        if (verbose)
            std::cout << "ADMMfast iter=" << iter << " prim=" << r_prim << " dual=" << r_dual << "  rho=" << rho_i
                      << " alpha=" << nalpha << "  tols=" << this->tol_prim << " " << this->tol_dual << std::endl;

        // For recording into violation history, if debugging
        if (this->record_violation_history) {
            // combined residual
            // double r_combined = ((z - l).cwiseProduct(S)).norm() + ((l - l_old).cwiseQuotient(S)).norm();
            // double r_combined_pre = (z - l).norm() + (l - l_old).norm(); //combined res.in precond.metric
            double c_k_p = (1 / rho) * (y - y_old).norm() + rho * (l - l_old).norm();
            AtIterationEnd(c_k_p, (l - l_old).norm(), iter);
        }

        // Termination:
        if ((r_prim < this->tol_prim) && (r_dual < this->tol_dual)) {
            if (verbose)
                std::cout << "ADMMfast converged ok! at iter=" << iter << std::endl;
            break;
        }

        // Nesterov acceleration
        if (iter > 1) {
            double c_k_p = (1 / rho) * (y - y_old).norm() + rho * (l - l_old).norm();
            if (c_k_p < eta * c_k) {
                double nalpha_p = (1.0 + std::sqrt(1.0 + 4 * std::pow(nalpha, 2))) / (2.0);
                l = l + ((nalpha - 1.0) / (nalpha_p)) * (l - l_old);
                v = v + ((nalpha - 1.0) / (nalpha_p)) * (v - v_old);  // trick to avoid recomputing it at the end
                y = y + ((nalpha - 1.0) / (nalpha_p)) * (y - y_old);
                nalpha = nalpha_p;
                c_k = c_k_p;
            } else {
                if (verbose)
                    std::cout << "ADMMfast Nesterov restart." << iter << std::endl;
                nalpha = 1.0;
                c_k = (1 / eta) * c_k_p;
            }
        }

        // once in a while update the rho step parameter
        if ((iter % this->stepadjust_each) == 0) {
            double rhofactor = 1;  // default do not shrink / enlarge

            if (this->stepadjust_type == AdmmStepType::NONE) {
                rhofactor = 1.;
            }

            if (this->stepadjust_type == AdmmStepType::BALANCED_UNSCALED) {
                rhofactor = std::sqrt(r_prim_pre / r_dual_pre);
            }

            if (this->stepadjust_type == AdmmStepType::BALANCED_FAST) {
                double r_prim_scaled =
                    r_prim_pre / (std::max(z.lpNorm<Eigen::Infinity>(), l.lpNorm<Eigen::Infinity>()) +
                                  1e-10);  // maybe norm(l, inf) very similar to norm(z, inf)
                double r_dual_scaled =
                    r_dual_pre /
                    (y.lpNorm<Eigen::Infinity>() +
                     1e-10);  //  as in "ADMM Penalty Parameter Selection by Residual Balancing", Brendt Wohlberg
                rhofactor = std::sqrt(r_prim_scaled / (r_dual_scaled + 1e-10));
            }

            if (this->stepadjust_type == AdmmStepType::BALANCED_RANGE) {
                double r_prim_scaled = r_prim / this->tol_prim;
                double r_dual_scaled = r_dual / this->tol_dual;
                rhofactor = std::sqrt(r_prim_scaled / (r_dual_scaled + 1e-10));
            }

            // safeguards against extreme shrinking
            if (rhofactor < 1.0 / this->stepadjust_maxfactor) {
                rhofactor = 1.0 / this->stepadjust_maxfactor;
            }
            if (rhofactor > this->stepadjust_maxfactor) {
                rhofactor = this->stepadjust_maxfactor;
            }

            if ((rhofactor > this->stepadjust_threshold) || (rhofactor < 1.0 / this->stepadjust_threshold)) {
                ChTimer m_timer_refactorize;
                m_timer_refactorize.start();

                // Avoid rebuilding all sparse matrix:
                // A) just remove old rho with -= :
                for (int i = 0; i < nc; ++i)
                    LS_solver->A().coeffRef(nv + i, nv + i) -= -(sigma + vrho(i));

                // Update rho
                rho_i = rho_i * rhofactor;

                // vrho(fric == -2) = rho_b; //  special step for bilateral joints
                vrho.setConstant(rho_i);
                s_c = 0;
                for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
                    if (mconstraints[ic]->IsActive()) {
                        if (mconstraints[ic]->GetMode() == ChConstraint::Mode::LOCK)
                            vrho(s_c) = rho_b;
                        s_c++;
                    }
                }

                // UPDATE FACTORIZATION
                //
                //  A = [M, Cq'; Cq, -diag(vsigma+vrho) + E ];
                //
                // To avoid rebuilding A, we just removed the rho step from the diagonal in A), and now:
                // B) add old rho with += :
                for (int i = 0; i < nc; ++i)
                    LS_solver->A().coeffRef(nv + i, nv + i) += -(sigma + vrho(i));

                LS_solver->SetupCurrent();  // LU decomposition ++++++++++++++++++++++++++++++++++++++

                m_timer_refactorize.stop();
                if (verbose)
                    std::cout << " Time for re-factorize : << " << m_timer_refactorize.GetTimeSeconds() << "s"
                              << std::endl;
            }

        }  // end step adjust

    }  // end iteration

    l = l.cwiseProduct(S);

    /*
    std::cout << "Y resulting:" << std::endl;
        for (int k = 0; k < std::min(y.rows(), 10); ++k)
            std::cout << "  " << y(k) << std::endl;
    */

    sysd.FromVectorToConstraints(l);
    sysd.FromVectorToVariables(v);

    return r_dual;
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

class ChSolverADMM_StepType_enum_mapper : public ChSolverADMM {
  public:
    CH_ENUM_MAPPER_BEGIN(AdmmStepType);
    CH_ENUM_VAL(NONE);
    CH_ENUM_VAL(BALANCED_UNSCALED);
    CH_ENUM_VAL(BALANCED_FAST);
    CH_ENUM_VAL(BALANCED_RANGE);
    CH_ENUM_MAPPER_END(AdmmStepType);
};

void ChSolverADMM::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChSolverADMM>();
    // serialize parent class
    ChIterativeSolverVI::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(precond);
    archive_out << CHNVP(rho);
    archive_out << CHNVP(rho_b);
    archive_out << CHNVP(sigma);
    archive_out << CHNVP(stepadjust_each);
    archive_out << CHNVP(stepadjust_threshold);
    archive_out << CHNVP(stepadjust_maxfactor);
    archive_out << CHNVP(tol_prim);
    archive_out << CHNVP(tol_dual);
    ChSolverADMM_StepType_enum_mapper::AdmmStepType_mapper mmapper;
    archive_out << CHNVP(mmapper(this->stepadjust_type), "stepadjust_type");
}

void ChSolverADMM::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChSolverADMM>();
    // deserialize parent class
    ChIterativeSolverVI::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(precond);
    archive_in >> CHNVP(rho);
    archive_in >> CHNVP(rho_b);
    archive_in >> CHNVP(sigma);
    archive_in >> CHNVP(stepadjust_each);
    archive_in >> CHNVP(stepadjust_threshold);
    archive_in >> CHNVP(stepadjust_maxfactor);
    archive_in >> CHNVP(tol_prim);
    archive_in >> CHNVP(tol_dual);
    ChSolverADMM_StepType_enum_mapper::AdmmStepType_mapper mmapper;
    archive_in >> CHNVP(mmapper(this->stepadjust_type), "stepadjust_type");
}

}  // end namespace chrono
