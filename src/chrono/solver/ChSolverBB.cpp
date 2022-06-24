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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/solver/ChSolverBB.h"
#include "chrono/core/ChMathematics.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverBB)

ChSolverBB::ChSolverBB() : n_armijo(10), max_armijo_backtrace(3), lastgoodres(1e30) {}

double ChSolverBB::Solve(ChSystemDescriptor& sysd) {
    std::vector<ChConstraint*>& mconstraints = sysd.GetConstraintsList();
    std::vector<ChVariables*>& mvariables = sysd.GetVariablesList();

    if (sysd.GetKblocksList().size() > 0) {
        std::cerr << "\n\nChSolverBB: Can NOT use Barzilai-Borwein solver if there are stiffness matrices.\n" << std::endl;
        throw ChException("ChSolverBB: Do NOT use Barzilai-Borwein solver if there are stiffness matrices.");
    }

    // Tuning of the spectral gradient search
    double a_min = 1e-13;
    double a_max = 1e13;
    double sigma_min = 0.1;
    double sigma_max = 0.9;
    double alpha = 0.0001;
    double gamma = 1e-4;
    double gdiff = 0.000001;

    bool do_BB1e2 = true;
    bool do_BB1 = false;
    bool do_BB2 = false;
    double neg_BB1_fallback = 0.11;
    double neg_BB2_fallback = 0.12;

    m_iterations = 0;
    // Allocate auxiliary vectors;

    int nc = sysd.CountActiveConstraints();
    if (verbose)
        GetLog() << "\n-----Barzilai-Borwein, solving nc=" << nc << "unknowns \n";

    ChVectorDynamic<> ml(nc);
    ChVectorDynamic<> ml_candidate(nc);
    ChVectorDynamic<> mg(nc);
    ChVectorDynamic<> mg_p(nc);
    ChVectorDynamic<> ml_p(nc);
    ChVectorDynamic<> mdir(nc);
    ChVectorDynamic<> mb(nc);
    ChVectorDynamic<> mb_tmp(nc);
    ChVectorDynamic<> ms(nc);
    ChVectorDynamic<> my(nc);
    ChVectorDynamic<> mD(nc);
    ChVectorDynamic<> mDg(nc);

    // Update auxiliary data in all constraints before starting,
    // that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        mconstraints[ic]->Update_auxiliary();

    // Average all g_i for the triplet of contact constraints n,u,v.
    //  Can be used for the fixed point phase and/or by preconditioner.
    int j_friction_comp = 0;
    double gi_values[3];
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
        if (mconstraints[ic]->GetMode() == CONSTRAINT_FRIC) {
            gi_values[j_friction_comp] = mconstraints[ic]->Get_g_i();
            j_friction_comp++;
            if (j_friction_comp == 3) {
                double average_g_i = (gi_values[0] + gi_values[1] + gi_values[2]) / 3.0;
                mconstraints[ic - 2]->Set_g_i(average_g_i);
                mconstraints[ic - 1]->Set_g_i(average_g_i);
                mconstraints[ic - 0]->Set_g_i(average_g_i);
                j_friction_comp = 0;
            }
        }
    }
    // The vector with the diagonal of the N matrix
    mD.setZero();
    int d_i = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        if (mconstraints[ic]->IsActive()) {
            mD(d_i, 0) = mconstraints[ic]->Get_g_i();
            ++d_i;
        }

    // ***TO DO*** move the following thirty lines in a short function ChSystemDescriptor::ShurBvectorCompute() ?

    // Compute the b_shur vector in the Shur complement equation N*l = b_shur
    // with
    //   N_shur  = D'* (M^-1) * D
    //   b_shur  = - c + D'*(M^-1)*k = b_i + D'*(M^-1)*k
    // but flipping the sign of lambdas,  b_shur = - b_i - D'*(M^-1)*k
    // Do this in three steps:

    // Put (M^-1)*k    in  q  sparse vector of each variable..
    for (unsigned int iv = 0; iv < mvariables.size(); iv++)
        if (mvariables[iv]->IsActive())
            mvariables[iv]->Compute_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb());  // q = [M]'*fb

    // ...and now do  b_shur = - D'*q = - D'*(M^-1)*k ..
    mb.setZero();
    int s_i = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        if (mconstraints[ic]->IsActive()) {
            mb(s_i, 0) = -mconstraints[ic]->Compute_Cq_q();
            ++s_i;
        }

    // ..and finally do   b_shur = b_shur - c
    sysd.BuildBiVector(mb_tmp);  // b_i   =   -c   = phi/h
    mb -= mb_tmp;

    // Optimization: backup the  q  sparse data computed above,
    // because   (M^-1)*k   will be needed at the end when computing primals.
    ChVectorDynamic<> mq;
    sysd.FromVariablesToVector(mq, true);

    // Initialize lambdas
    if (m_warm_start)
        sysd.FromConstraintsToVector(ml);
    else
        ml.setZero();

    // Initial projection of ml   ***TO DO***?
    sysd.ConstraintsProject(ml);

    // Fallback solution
    lastgoodres = 1e30;
    ml_candidate = ml;

    // g = gradient of 0.5*l'*N*l-l'*b
    // g = N*l-b
    sysd.ShurComplementProduct(mg, ml);  // 1)  g = N * l
    mg -= mb;                            // 2)  g = N * l - b_shur

    mg_p = mg;

    //
    // THE LOOP
    //

    double mf_p = 0;
    double mf = 1e29;
    std::vector<double> f_hist;

    for (int iter = 0; iter < m_max_iterations; iter++) {
        // Dg = Di*g;
        mDg = mg;
        if (m_use_precond)
            mDg = mDg.array() / mD.array();

        // dir  = [P(l - alpha*Dg) - l]
        mdir = ml - alpha * mDg;        // dir = l - alpha*Dg
        sysd.ConstraintsProject(mdir);  // dir = P(l - alpha*Dg)
        mdir -= ml;                     // dir = P(l - alpha*Dg) - l

        // dTg = dir'*g;
        double dTg = mdir.dot(mg);

        // BB dir backward!? fallback to nonpreconditioned dir
        if (dTg > 1e-8) {
            // dir  = [P(l - alpha*g) - l]
            mdir = ml - alpha * mg;         // dir = l - alpha*g
            sysd.ConstraintsProject(mdir);  // dir = P(l - alpha*g) ...
            mdir -= ml;                     // dir = P(l - alpha*g) - l
            // dTg = d'*g;
            dTg = mdir.dot(mg);
        }

        double lambda = 1;

        int n_backtracks = 0;
        bool armijo_repeat = true;

        while (armijo_repeat) {
            // l_p = l + lambda*dir;
            ml_p = ml + lambda * mdir;

            // m_tmp = Nl_p = N*l_p;
            sysd.ShurComplementProduct(mb_tmp, ml_p);

            // g_p = N * l_p - b  = Nl_p - b
            mg_p = mb_tmp - mb;

            // f_p = 0.5*l_p'*N*l_p - l_p'*b  = l_p'*(0.5*Nl_p - b);
            mf_p = ml_p.dot(0.5 * mb_tmp - mb);

            f_hist.push_back(mf_p);

            double max_compare = 10e29;
            for (int h = 1; h <= ChMin(iter, this->n_armijo); h++) {
                double compare = f_hist[iter - h] + gamma * lambda * dTg;
                if (compare > max_compare)
                    max_compare = compare;
            }

            if (mf_p > max_compare) {
                armijo_repeat = true;
                if (iter > 0)
                    mf = f_hist[iter - 1];
                double lambdanew = -lambda * lambda * dTg / (2 * (mf_p - mf - lambda * dTg));
                lambda = ChMax(sigma_min * lambda, ChMin(sigma_max * lambda, lambdanew));
                if (verbose)
                    GetLog() << " Repeat Armijo, new lambda=" << lambda << "\n";
            } else {
                armijo_repeat = false;
            }

            n_backtracks = n_backtracks + 1;
            if (n_backtracks > this->max_armijo_backtrace)
                armijo_repeat = false;
        }

        ms = ml_p - ml;  // s = l_p - l;
        my = mg_p - mg;  // y = g_p - g;
        ml = ml_p;       // l = l_p;
        mg = mg_p;       // g = g_p;

        if (((do_BB1e2) && (iter % 2 == 0)) || do_BB1) {
            if (m_use_precond)
                mb_tmp = ms.array() * mD.array();
            else
                mb_tmp = ms;
            double sDs = ms.dot(mb_tmp);
            double sy = ms.dot(my);
            if (sy <= 0) {
                alpha = neg_BB1_fallback;
            } else {
                double alph = sDs / sy;  // (s,Ds)/(s,y)   BB1
                alpha = ChMin(a_max, ChMax(a_min, alph));
            }
        }

        /*
        // this is a modified rayleight quotient - looks like it works anyway...
        if (((do_BB1e2) && (iter%2 ==0)) || do_BB1)
        {
            double ss = ms.MatrDot(ms,ms);
            mb_tmp = my;
            if (m_use_precond)
                mb_tmp.MatrDivScale(mD);
            double sDy = ms.MatrDot(ms, mb_tmp);
            if (sDy <= 0)
            {
                alpha = neg_BB1_fallback;
            }
            else
            {
                double alph = ss / sDy;  // (s,s)/(s,Di*y)   BB1 (modified version)
                alpha = ChMin (a_max, ChMax(a_min, alph));
            }
        }
        */

        if (((do_BB1e2) && (iter % 2 != 0)) || do_BB2) {
            double sy = ms.dot(my);
            if (m_use_precond)
                mb_tmp = my.array() / mD.array();
            else
                mb_tmp = my;
            double yDy = my.dot(mb_tmp);
            if (sy <= 0) {
                alpha = neg_BB2_fallback;
            } else {
                double alph = sy / yDy;  // (s,y)/(y,Di*y)   BB2
                alpha = ChMin(a_max, ChMax(a_min, alph));
            }
        }

        // Project the gradient (for rollback strategy)
        // g_proj = (l-project_orthogonal(l - gdiff*g, fric))/gdiff;
        mb_tmp = ml - gdiff * mg;
        sysd.ConstraintsProject(mb_tmp);     // mb_tmp = ProjectionOperator(l - gdiff * g)
        mb_tmp = (ml - mb_tmp) / gdiff;      // mb_tmp = [l - ProjectionOperator(l - gdiff * g)] / gdiff
        double g_proj_norm = mb_tmp.norm();  // infinity norm is faster..

        // Rollback solution: the last best candidate ('l' with lowest projected gradient)
        // in fact the method is not monotone and it is quite 'noisy', if we do not
        // do this, a prematurely truncated iteration might give a crazy result.
        if (g_proj_norm < lastgoodres) {
            lastgoodres = g_proj_norm;
            ml_candidate = ml;
        }

        // METRICS - convergence, plots, etc

        double maxdeltalambda = ms.lpNorm<Eigen::Infinity>();
        double maxd = lastgoodres;

        // For recording into correction/residuals/violation history, if debugging
        if (this->record_violation_history)
            AtIterationEnd(maxd, maxdeltalambda, iter);

        if (verbose)
            GetLog() << "  iter=" << iter << "   f=" << mf_p << "  |d|=" << maxd << "  |s|=" << maxdeltalambda << "\n";

        m_iterations++;

        // Terminate the loop if violation in constraints has been successfully limited.
        // ***TO DO*** a reliable termination criterion..
        /*
        if (maxd < m_tolerance)
        {
            GetLog() <<"BB premature proj.gradient break at i=" << iter << "\n";
            break;
        }
        */
    }

    // Fallback to best found solution (might be useful because of nonmonotonicity)
    ml = ml_candidate;

    // Resulting DUAL variables:
    // store ml temporary vector into ChConstraint 'l_i' multipliers
    sysd.FromVectorToConstraints(ml);

    // Resulting PRIMAL variables:
    // compute the primal variables as   v = (M^-1)(k + D*l)

    // v = (M^-1)*k  ...    (by rewinding to the backup vector computed ad the beginning)
    sysd.FromVectorToVariables(mq);

    // ... + (M^-1)*D*l     (this increment and also stores 'qb' in the ChVariable items)
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
        if (mconstraints[ic]->IsActive())
            mconstraints[ic]->Increment_q(mconstraints[ic]->Get_l_i());
    }

    if (verbose)
        GetLog() << "-----\n";

    return lastgoodres;
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

void ChSolverBB::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSolverBB>();
    // serialize parent class
    ChIterativeSolverVI::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(n_armijo);
    marchive << CHNVP(max_armijo_backtrace);
    marchive << CHNVP(m_use_precond);
}

void ChSolverBB::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChSolverBB>();
    // deserialize parent class
    ChIterativeSolverVI::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(n_armijo);
    marchive >> CHNVP(max_armijo_backtrace);
    marchive >> CHNVP(m_use_precond);
}

}  // end namespace chrono
