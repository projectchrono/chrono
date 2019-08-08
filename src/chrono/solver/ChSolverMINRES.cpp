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

#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/solver/ChConstraintTwoTuplesFrictionT.h"
#include "chrono/core/ChMathematics.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverMINRES)

ChSolverMINRES::ChSolverMINRES(int mmax_iters, bool mwarm_start, double mtolerance)
    : ChIterativeSolver(mmax_iters, mwarm_start, mtolerance, 0.2),
      rel_tolerance(0.0),
      feas_tolerance(0.2),
      max_fixedpoint_steps(6),
      diag_preconditioning(true) {}

double ChSolverMINRES::Solve(ChSystemDescriptor& sysd) {
    std::vector<ChConstraint*>& mconstraints = sysd.GetConstraintsList();
    std::vector<ChVariables*>& mvariables = sysd.GetVariablesList();

    // If stiffness blocks are used, the Schur complement cannot be esily
    // used, so fall back to the Solve_SupportingStiffness method, that operates on KKT.
    if (sysd.GetKblocksList().size() > 0)
        return Solve_SupportingStiffness(sysd);

    tot_iterations = 0;
    double maxviolation = 0.;
    int i_friction_comp = 0;
    // int iter_tot = 0;	// replaced with tot_iterations - Hammad

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

    // Allocate auxiliary vectors;

    int nc = sysd.CountActiveConstraints();

    if (verbose)
        GetLog() << "nc = " << nc << "\n";
    ChVectorDynamic<> ml(nc);
    ChVectorDynamic<> mb(nc);
    ChVectorDynamic<> mr(nc);
    ChVectorDynamic<> mp(nc);
    ChVectorDynamic<> mb_i(nc);
    ChVectorDynamic<> Nr(nc);
    ChVectorDynamic<> Np(nc);
    std::vector<bool> en_l(nc);

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

    // ...and now do  b_shur = - D' * q  ..
    mb.setZero();
    int s_i = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        if (mconstraints[ic]->IsActive()) {
            mb(s_i, 0) = -mconstraints[ic]->Compute_Cq_q();
            ++s_i;
        }

    // ..and finally do   b_shur = b_shur - c
    sysd.BuildBiVector(mb_i);  // b_i   =   -c   = phi/h
    mb -= mb_i;

    // Optimization: backup the  q  sparse data computed above,
    // because   (M^-1)*k   will be needed at the end when computing primals.
    ChVectorDynamic<> mq;
    sysd.FromVariablesToVector(mq, true);

    // Initialize lambdas
    if (warm_start)
        sysd.FromConstraintsToVector(ml);
    else
        ml.setZero();

    // Initially all constraints are enabled
    for (int ie = 0; ie < nc; ie++)
        en_l[ie] = true;

    //
    // THE LOOP
    //

    while (true) {
        if (verbose)
            GetLog() << "\n";

        //
        // A)  The MINRES loop. Operates only on set defined by en_l
        //

        // Compute initial residual, with minus sign
        sysd.ShurComplementProduct(mr, ml, &en_l);  // r = N*l
        mr = mb - mr;                               // r = - N*l + b_shur

        for (int row = 0; row < nc; row++)
            if (en_l[row] == false)
                mr(row) = 0;

        mp = mr;

        sysd.ShurComplementProduct(Nr, mr, &en_l);  // Nr  =  N * r
        Np = Nr;                                    // Np  =  N * p

        while (true) {
            if (verbose)
                GetLog() << "K";

            ///sysd.ShurComplementProduct(Nr, mr, &en_l); // Nr  =  N * r  (no, recompute only when mr changes, see later)
            double rNr = mr.dot(Nr);  // rNr = r' * N * r

            ///sysd.ShurComplementProduct(Np, mp, &en_l); // Np  =  N * p  (no, see optimization at the end)
            double den = Np.squaredNorm();

            if (den == 0)
                break;

            double alpha = rNr / den;  // alpha = r'*N*r / ((N*p)'(N*p))

            ///ml += mp*alpha;  // l = l + alpha * p;  done below, also with projection

            // btw. must split projection in two loops to avoid troubles with frictional contacts (three updates, but
            // project only at the end)
            double norm_corr = 0;
            double norm_jump = 0;
            double norm_viol = 0;
            double norm_dlam = 0;
            int s_cc = 0;
            for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
                if (mconstraints[ic]->IsActive()) {
                    if (en_l[s_cc] == true) {
                        double old_l = ml(s_cc);
                        double new_l = old_l + alpha * mp(s_cc);

                        mconstraints[ic]->Set_l_i(new_l);
                    }
                    ++s_cc;
                }
            s_cc = 0;
            for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
                if (mconstraints[ic]->IsActive()) {
                    if (en_l[s_cc] == true) {
                        double old_l = ml(s_cc);
                        double new_l = old_l + alpha * mp(s_cc);

                        mconstraints[ic]->Project();
                        double new_lp = mconstraints[ic]->Get_l_i();

                        double violation = mconstraints[ic]->Violation(mr(s_cc));  //?
                        if (mconstraints[ic]->GetMode() == CONSTRAINT_FRIC)
                            violation = fabs(ChMin(0.0, violation));

                        // ??? trouble with Tang. constraints, for the moment just disable norms
                        if (!dynamic_cast<ChConstraintTwoTuplesFrictionTall*>(mconstraints[ic])) {
                            norm_corr += pow(new_lp - new_l, 2);
                            norm_jump += pow(new_l - old_l, 2);
                            norm_dlam += pow(new_lp - old_l, 2);
                            norm_viol += pow(violation, 2);
                        }
                    }
                    ++s_cc;
                }
            norm_corr = sqrt(norm_corr);
            norm_jump = sqrt(norm_jump);
            norm_dlam = sqrt(norm_dlam);
            norm_viol = sqrt(norm_viol);

            if (norm_corr > this->feas_tolerance * norm_jump) {
                if (verbose)
                    GetLog() << " " << (norm_corr / norm_jump);
                break;
            }

            s_cc = 0;
            for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
                if (mconstraints[ic]->IsActive()) {
                    if (en_l[s_cc] == true) {
                        ml(s_cc) = mconstraints[ic]->Get_l_i();
                    }
                    ++s_cc;
                }

            mr -= alpha * Np;  // r = r - alpha * N*p;

            sysd.ShurComplementProduct(Nr, mr, &en_l);  // Nr  =  N * r
            double rNr_ = mr.dot(Nr);                   // rNr = r' * N * r

            double beta = rNr_ / rNr;  // beta = r'*(N*r)/ rjNrj;

            mp = mr + beta * mp;  // p = r + beta*p;
            Np = Nr + beta * Np;  // N*p' = Nr + beta * N*p

            // For recording into violation history
            if (this->record_violation_history)
                AtIterationEnd(
                    0.0, norm_dlam,
                    tot_iterations);  //(norm_viol, norm_dlam, tot_iterations); ***DEBUG*** use 0.0 to show phase

            ++tot_iterations;
            if (tot_iterations > this->max_iterations)
                break;
        }

        if (tot_iterations > this->max_iterations)
            break;

        if (verbose)
            GetLog() << "\n";

        //
        // B)  The FIXED POINT, it also will find active sets. Operates on entire set
        //

        for (int iter_fixedp = 0; iter_fixedp < this->max_fixedpoint_steps; iter_fixedp++) {
            if (verbose)
                GetLog() << "p";

            // Compute residual  as  r = N*l - b_shur
            sysd.ShurComplementProduct(mr, ml, 0);  // 1)  r = N*l ...
            mr -= mb;                               // 2)  r = N*l - b_shur

            //	l = l - omega * (diag(N)^-1)* (res);
            double norm_dlam = 0;
            double norm_viol = 0;
            // must split projection in two loops to avoid troubles with frictional contacts (three updates, but project
            // only at the end)
            int s_cc = 0;
            for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
                if (mconstraints[ic]->IsActive()) {
                    double dlam = -mr(s_cc) * (this->omega / mconstraints[ic]->Get_g_i());
                    ml(s_cc) += dlam;
                    mconstraints[ic]->Set_l_i(ml(s_cc));
                    norm_dlam += pow(dlam, 2);
                    ++s_cc;
                }
            s_cc = 0;
            for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
                if (mconstraints[ic]->IsActive()) {
                    mconstraints[ic]->Project();
                    ml(s_cc) = mconstraints[ic]->Get_l_i();

                    double violation = mconstraints[ic]->Violation(mr(s_cc));
                    if (mconstraints[ic]->GetMode() == CONSTRAINT_FRIC)
                        violation = fabs(ChMin(0.0, violation));

                    norm_viol += pow(violation, 2);

                    ++s_cc;
                }
            norm_dlam = sqrt(norm_dlam);
            norm_viol = sqrt(norm_viol);

            // For recording into violation history
            if (this->record_violation_history)
                AtIterationEnd(
                    1.0, norm_dlam,
                    tot_iterations);  //(norm_viol, norm_dlam, tot_iterations); ***DEBUG*** use 1.0 to show phase

            ++tot_iterations;
            if (tot_iterations > this->max_iterations)
                break;
        }

        if (tot_iterations > this->max_iterations)
            break;

        if (verbose)
            GetLog() << "\n";
        //
        // C)  The ACTIVE SET detection
        //

        for (int row = 0; row < nc; row++) {
            if (ml(row) == 0) {
                if (verbose)
                    GetLog() << "0";
                en_l[row] = false;
            } else {
                if (verbose)
                    GetLog() << "1";
                en_l[row] = true;
            }
        }
    }

    if (verbose)
        GetLog() << "-----\n";

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

    return maxviolation;
}

// ---------------------------------------------------------------------------

double ChSolverMINRES::Solve_SupportingStiffness(ChSystemDescriptor& sysd) {
    bool do_preconditioning = this->diag_preconditioning;

    std::vector<ChConstraint*>& mconstraints = sysd.GetConstraintsList();
    std::vector<ChVariables*>& mvariables = sysd.GetVariablesList();
    std::vector<ChKblock*>& mstiffness = sysd.GetKblocksList();

    this->tot_iterations = 0;

    // Allocate auxiliary vectors;

    int nv = sysd.CountActiveVariables();
    int nc = sysd.CountActiveConstraints();
    int nx = nv + nc;  // total scalar unknowns, in x vector for full KKT system Z*x-d=0

    if (verbose)
        GetLog() << "\n----- MINRES -supporting stiffness-, n.vars nx=" << nx << "  max.iters=" << max_iterations
                 << "\n";

    ChVectorDynamic<> x(nx);
    ChVectorDynamic<> d(nx);
    ChVectorDynamic<> p(nx);
    ChVectorDynamic<> r(nx);
    ChVectorDynamic<> Zr(nx);
    ChVectorDynamic<> Zp(nx);
    ChVectorDynamic<> MZp(nx);
    ChVectorDynamic<> r_old(nx);
    ChVectorDynamic<> Zr_old(nx);

    ChVectorDynamic<> tmp(nx);
    ChVectorDynamic<> mDi(nx);

    this->tot_iterations = 0;
    double maxviolation = 0.;

    //
    // --- Compute a diagonal (scaling) preconditioner for the KKT system:
    //

    // Initialize the mDi vector with the diagonal of the Z matrix
    sysd.BuildDiagonalVector(mDi);

    // Pre-invert the values, to avoid wasting time with divisions in the following.
    // From now, mDi contains the inverse of the diagonal of Z.
    // Note, for constraints, the diagonal is 0, so set inverse of D as 1 assuming
    // a constraint preconditioning and assuming the dot product of jacobians is already about 1.
    for (int nel = 0; nel < mDi.size(); nel++) {
        if (fabs(mDi(nel)) > 1e-9)
            mDi(nel) = 1.0 / mDi(nel);
        else
            mDi(nel) = 1.0;
    }

    //
    // --- Vector initialization and book-keeping
    //

    // Initialize the x vector of unknowns x ={q; -l} (if warm starting needed, initialize
    // x with current values of q and l in variables and constraints)
    if (warm_start)
        sysd.FromUnknownsToVector(x);
    else
        x.setZero();

    // Initialize the d vector filling it with {f, -b}
    sysd.BuildDiVector(d);

    //
    // --- THE P-MINRES ALGORITHM
    //

    double rel_tol = this->rel_tolerance;
    double abs_tol = this->tolerance;
    double rel_tol_d = d.lpNorm<Eigen::Infinity>() * rel_tol;

    // r = d - Z*x;
    sysd.SystemProduct(r, x);  // r = Z*x
    r = d - r;                 // r =-Z*x+d

    // Precondition: r = M(r)
    if (do_preconditioning)
        r = r.array() * mDi.array();

    // p = r
    p = r;

    // Zr = Z*r;
    sysd.SystemProduct(Zr, r);

    // Zp = Z*p;
    Zp = Zr;

    //
    // THE LOOP
    //

    for (int iter = 0; iter < max_iterations; iter++) {
        // Terminate iteration when the projected r is small, if (norm(r,2) <= max(rel_tol_d,abs_tol))
        double r_proj_resid = r.norm();
        if (r_proj_resid < ChMax(rel_tol_d, abs_tol)) {
            if (verbose)
                GetLog() << "P(r)-converged! iter=" << iter << " |P(r)|=" << r_proj_resid << "\n";
            break;
        }

        // MZp = M*Z*p
        if (do_preconditioning)
            MZp = Zp.array() * mDi.array();
        else
            MZp = Zp;

        // alpha = (r' * Zr) / ((Zp)'*(MZp));
        double rZr = r.dot(Zr);      // z'* Zr
        double ZpMZp = Zp.dot(MZp);  // ZpMZp = (Zp)'*(MZp)

        double alpha = rZr / ZpMZp;

        // x = x + alpha * p;
        tmp = alpha * p;
        x += tmp;

        double maxdeltaunknowns = tmp.norm();

        // r_old = r;
        r_old = r;

        // r = r - alpha * MZp;
        r -= alpha * MZp;

        // Zr_old = Zr;
        Zr_old = Zr;

        // Zr = Z*r;
        sysd.SystemProduct(Zr, r);

        // beta = (r' * N*r) / (r_old' * N*r_old);
        double numerator = r.dot(Zr);            // r'* Z *r
        double denominator = r_old.dot(Zr_old);  // r_old'* Z *r_old
        double beta = numerator / denominator;

        // p = r + beta * p;
        p = r + beta * p;

        // Zp = Zr + beta * Zp;  % avoids multiply for Np=N*p
        Zp = Zr + beta * Zp;

        // ---------------------------------------------
        // METRICS - convergence, plots, etc

        // For recording into correction/residuals/violation history, if debugging
        if (this->record_violation_history)
            AtIterationEnd(r_proj_resid, maxdeltaunknowns, iter);
    }

    // After having solved for unknowns x={q;-l}, now copy those values from x vector to
    // the q values in ChVariable items and to l values in ChConstraint items
    sysd.FromVectorToUnknowns(x);

    if (verbose)
        GetLog() << "MINRES residual: " << r.norm() << " ---\n";

    return maxviolation;
}

void ChSolverMINRES::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSolverMINRES>();
    // serialize parent class
    ChIterativeSolver::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(feas_tolerance);
    marchive << CHNVP(max_fixedpoint_steps);
    marchive << CHNVP(diag_preconditioning);
    marchive << CHNVP(rel_tolerance);
}

void ChSolverMINRES::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChSolverMINRES>();
    // deserialize parent class
    ChIterativeSolver::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(feas_tolerance);
    marchive >> CHNVP(max_fixedpoint_steps);
    marchive >> CHNVP(diag_preconditioning);
    marchive >> CHNVP(rel_tolerance);
}

}  // end namespace chrono
