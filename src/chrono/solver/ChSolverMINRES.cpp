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

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverMINRES)

double ChSolverMINRES::Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
) {
    std::vector<ChConstraint*>& mconstraints = sysd.GetConstraintsList();
    std::vector<ChVariables*>& mvariables = sysd.GetVariablesList();

    // If stiffness blocks are used, the Schur complement cannot be esily
    // used, so fall back to the Solve_SupportingStiffness method, that operates on KKT.
    if (sysd.GetKblocksList().size() > 0)
        return this->Solve_SupportingStiffness(sysd);

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
    ChMatrixDynamic<> ml(nc, 1);
    ChMatrixDynamic<> mb(nc, 1);
    ChMatrixDynamic<> mr(nc, 1);
    ChMatrixDynamic<> mp(nc, 1);
    ChMatrixDynamic<> mb_i(nc, 1);
    ChMatrixDynamic<> Nr(nc, 1);
    ChMatrixDynamic<> Np(nc, 1);
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
    mb.Reset();
    int s_i = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        if (mconstraints[ic]->IsActive()) {
            mb(s_i, 0) = -mconstraints[ic]->Compute_Cq_q();
            ++s_i;
        }

    // ..and finally do   b_shur = b_shur - c
    sysd.BuildBiVector(mb_i);  // b_i   =   -c   = phi/h
    mb.MatrDec(mb_i);

    // Optimization: backup the  q  sparse data computed above,
    // because   (M^-1)*k   will be needed at the end when computing primals.
    ChMatrixDynamic<> mq;
    sysd.FromVariablesToVector(mq, true);

    // Initialize lambdas
    if (warm_start)
        sysd.FromConstraintsToVector(ml);
    else
        ml.FillElem(0);

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
        sysd.ShurComplementProduct(mr, &ml, &en_l);  // 1)  r = N*l ...
        mr.MatrDec(mb);                              // 2)  r = N*l - b_shur ...
        mr.MatrNeg();                                // 3)  r = - N*l + b_shur

        for (int row = 0; row < nc; row++)
            if (en_l[row] == false)
                mr(row) = 0;

        mp = mr;

        sysd.ShurComplementProduct(Nr, &mr, &en_l);  // Nr  =  N * r
        Np = Nr;                                     // Np  =  N * p

        while (true) {
            if (verbose)
                GetLog() << "K";
            // sysd.ShurComplementProduct(Nr, &mr,&en_l); // Nr  =  N * r  (no, recompute only when mr changes, see
            // later)
            double rNr = mr.MatrDot(mr, Nr);  // rNr = r' * N * r

            // sysd.ShurComplementProduct(Np, &mp,&en_l); // Np  =  N * p  (no, see optimization at the end)
            double den = pow(Np.NormTwo(), 2);  // den =  ((N*p)'(N*p))

            if (den == 0)
                break;

            double alpha = rNr / den;  // alpha = r'*N*r / ((N*p)'(N*p))

            // ml.MatrInc((mp*alpha));				// l = l + alpha * p;  done below, with projection too

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

            mr.MatrDec((Np * alpha));  // r = r - alpha * N*p;

            sysd.ShurComplementProduct(Nr, &mr, &en_l);  // Nr  =  N * r
            double rNr_ = mr.MatrDot(mr, Nr);            // rNr = r' * N * r

            double beta = rNr_ / rNr;  // beta = r'*(N*r)/ rjNrj;

            mp.MatrScale(beta);
            mp.MatrInc(mr);  // p = r + beta*p;

            Np.MatrScale(beta);  // Avoid matr x vector operation by doing:
            Np.MatrInc(Nr);      // N*p' = Nr + beta * N*p

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
            sysd.ShurComplementProduct(mr, &ml, 0);  // 1)  r = N*l ...
            mr.MatrDec(mb);                          // 2)  r = N*l - b_shur

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

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

double ChSolverMINRES::Solve_SupportingStiffness(
    ChSystemDescriptor& sysd  ///< system description with constraints and variables
) {
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

    ChMatrixDynamic<> x(nx, 1);
    ChMatrixDynamic<> d(nx, 1);
    ChMatrixDynamic<> p(nx, 1);
    ChMatrixDynamic<> r(nx, 1);
    ChMatrixDynamic<> Zr(nx, 1);
    ChMatrixDynamic<> Zp(nx, 1);
    ChMatrixDynamic<> MZp(nx, 1);
    ChMatrixDynamic<> r_old(nx, 1);
    ChMatrixDynamic<> Zr_old(nx, 1);

    ChMatrixDynamic<> tmp(nx, 1);
    ChMatrixDynamic<> mDi(nx, 1);

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
    for (int nel = 0; nel < mDi.GetRows(); nel++) {
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
        x.FillElem(0);

    // Initialize the d vector filling it with {f, -b}
    sysd.BuildDiVector(d);

    //
    // --- THE P-MINRES ALGORITHM
    //

    double rel_tol = this->rel_tolerance;
    double abs_tol = this->tolerance;
    double rel_tol_d = d.NormInf() * rel_tol;

    // r = d - Z*x;
    sysd.SystemProduct(
        r, &x);    // 1)  r = Z*x ...        #### MATR.MULTIPLICATION!!!### can be avoided if no warm starting!
    r.MatrNeg();   // 2)  r =-Z*x
    r.MatrInc(d);  // 3)  r =-Z*x+d

    // r = M(r)								//						   ## Precond
    if (do_preconditioning)
        r.MatrScale(mDi);

    // p = r
    p = r;

    // Zr = Z*r;
    sysd.SystemProduct(Zr, &r);  // 1)  Zr = Z*r ...        #### MATR.MULTIPLICATION!!!###

    // Zp = Z*p;
    Zp = Zr;

    //
    // THE LOOP
    //

    for (int iter = 0; iter < max_iterations; iter++) {
        // Terminate iteration when the projected r is small, if (norm(r,2) <= max(rel_tol_d,abs_tol))
        double r_proj_resid = r.NormTwo();
        if (r_proj_resid < ChMax(rel_tol_d, abs_tol)) {
            if (verbose)
                GetLog() << "P(r)-converged! iter=" << iter << " |P(r)|=" << r_proj_resid << "\n";
            break;
        }

        // MZp = M*Z*p
        MZp = Zp;
        if (do_preconditioning)
            MZp.MatrScale(mDi);

        // alpha = (r' * Zr) / ((Zp)'*(MZp));
        double rZr = r.MatrDot(r, Zr);      // 1)  z'* Zr
        double ZpMZp = r.MatrDot(Zp, MZp);  // 2)  ZpMZp = (Zp)'*(MZp)

        double alpha = rZr / ZpMZp;

        // x = x + alpha * p;
        tmp = p;
        tmp.MatrScale(alpha);
        x.MatrInc(tmp);

        double maxdeltaunknowns = tmp.NormTwo();

        // r_old = r;
        r_old = r;

        // r = r - alpha * MZp;
        tmp = MZp;
        tmp.MatrScale(-alpha);
        r.MatrInc(tmp);

        // Zr_old = Zr;
        Zr_old = Zr;

        // Zr = Z*r;
        sysd.SystemProduct(Zr, &r);  // 1)  Zr = Z*r ...        #### MATR.MULTIPLICATION!!!###

        // beta = (r' * N*r) / (r_old' * N*r_old);
        double numerator = r.MatrDot(r, Zr);            // 1)  r'* Z *r
        double denominator = r.MatrDot(r_old, Zr_old);  // 2)  r_old'* Z *r_old

        double beta = numerator / denominator;

        // p = r + beta * p;
        tmp = p;
        tmp.MatrScale(beta);
        p = r;
        p.MatrInc(tmp);

        // Zp = Zr + beta * Zp;  % avoids multiply for Np=N*p
        Zp.MatrScale(beta);
        Zp.MatrInc(Zr);

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
        GetLog() << "MINRES residual: " << r.NormTwo() << " ---\n";

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
