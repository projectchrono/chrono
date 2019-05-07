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

#include "chrono/solver/ChSolverPMINRES.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverPMINRES)

double ChSolverPMINRES::Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
) {
    bool do_preconditioning = this->diag_preconditioning;

    std::vector<ChConstraint*>& mconstraints = sysd.GetConstraintsList();
    std::vector<ChVariables*>& mvariables = sysd.GetVariablesList();

    // If stiffness blocks are used, the Schur complement cannot be esily
    // used, so fall back to the Solve_SupportingStiffness method, that operates on KKT.
    if (sysd.GetKblocksList().size() > 0)
        return this->Solve_SupportingStiffness(sysd);

    // Allocate auxiliary vectors;

    int nc = sysd.CountActiveConstraints();
    if (verbose)
        GetLog() << "\n-----Projected MINRES, solving nc=" << nc << "unknowns \n";

    ChMatrixDynamic<> ml(nc, 1);
    ChMatrixDynamic<> mb(nc, 1);
    ChMatrixDynamic<> mp(nc, 1);
    ChMatrixDynamic<> mr(nc, 1);
    ChMatrixDynamic<> mz(nc, 1);
    ChMatrixDynamic<> mz_old(nc, 1);
    ChMatrixDynamic<> mNp(nc, 1);
    ChMatrixDynamic<> mMNp(nc, 1);
    ChMatrixDynamic<> mNMr(nc, 1);
    ChMatrixDynamic<> mNMr_old(nc, 1);
    ChMatrixDynamic<> mtmp(nc, 1);
    ChMatrixDynamic<> mDi(nc, 1);

    this->tot_iterations = 0;
    double maxviolation = 0.;

    // Update auxiliary data in all constraints before starting,
    // that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        mconstraints[ic]->Update_auxiliary();

    // Average all g_i for the triplet of contact constraints n,u,v.
    //  Can be used as diagonal preconditioner.
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

    // The vector with the inverse of diagonal of the N matrix
    mDi.Reset();
    int d_i = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        if (mconstraints[ic]->IsActive()) {
            mDi(d_i, 0) = 1.0 / mconstraints[ic]->Get_g_i();
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

    // ...and now do  b_shur = - D' * q  ..
    mb.Reset();
    int s_i = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        if (mconstraints[ic]->IsActive()) {
            mb(s_i, 0) = -mconstraints[ic]->Compute_Cq_q();
            ++s_i;
        }

    // ..and finally do   b_shur = b_shur - c
    sysd.BuildBiVector(mtmp);  // b_i   =   -c   = phi/h
    mb.MatrDec(mtmp);

    // Optimization: backup the  q  sparse data computed above,
    // because   (M^-1)*k   will be needed at the end when computing primals.
    ChMatrixDynamic<> mq;
    sysd.FromVariablesToVector(mq, true);

    double rel_tol = this->rel_tolerance;
    double abs_tol = this->tolerance;
    double rel_tol_b = mb.NormInf() * rel_tol;

    // Initialize lambdas
    if (warm_start)
        sysd.FromConstraintsToVector(ml);
    else
        ml.FillElem(0);

    // Initial projection of ml   ***TO DO***?
    // ...

    // r = b - N*l;
    sysd.ShurComplementProduct(mr, &ml);  // 1)  r = N*l ... MATR.MULTIPLICATION!!!can be avoided if no warm starting!
    mr.MatrNeg();                         // 2)  r =-N*l
    mr.MatrInc(mb);                       // 3)  r =-N*l+b

    // r = (project_orthogonal(l+diff*r, fric) - l)/diff;
    mr.MatrScale(this->grad_diffstep);
    mr.MatrInc(ml);
    sysd.ConstraintsProject(mr);  // p = P(l+diff*p) ...
    mr.MatrDec(ml);
    mr.MatrScale(1.0 / this->grad_diffstep);  // p = (P(l+diff*p)-l)/diff

    // p = Mi * r;
    mp = mr;
    if (do_preconditioning)
        mp.MatrScale(mDi);

    // z = Mi * r;
    mz = mp;

    // NMr = N*M*r = N*z
    sysd.ShurComplementProduct(mNMr, &mz);  // NMr = N*z    #### MATR.MULTIPLICATION!!!###

    // Np = N*p
    sysd.ShurComplementProduct(mNp, &mp);  // Np = N*p    #### MATR.MULTIPLICATION!!!###

    //
    // THE LOOP
    //

    std::vector<double> f_hist;

    for (int iter = 0; iter < max_iterations; iter++) {
        // MNp = Mi*Np; % = Mi*N*p                  %% -- Precond
        mMNp = mNp;
        if (do_preconditioning)
            mMNp.MatrScale(mDi);

        // alpha = (z'*(NMr))/((MNp)'*(Np));
        double zNMr = mz.MatrDot(mz, mNMr);      // 1)  zMNr = z'* NMr
        double MNpNp = mMNp.MatrDot(mMNp, mNp);  // 2)  MNpNp = ((MNp)'*(Np))

        if (fabs(MNpNp) < 10e-30) {
            if (verbose)
                GetLog() << "Iter=" << iter << " Rayleigh quotient alpha breakdown: " << zNMr << " / " << MNpNp << "\n";
            MNpNp = 10e-12;
        }

        double alpha = zNMr / MNpNp;  // 3)  alpha = (z'*(NMr))/((MNp)'*(Np));

        // l = l + alpha * p;
        mtmp = mp;
        mtmp.MatrScale(alpha);
        ml.MatrInc(mtmp);

        double maxdeltalambda = mtmp.NormTwo();  //***better NormInf() for speed reasons?

        // l = Proj(l)
        sysd.ConstraintsProject(ml);  // l = P(l)

        // r = b - N*l;
        sysd.ShurComplementProduct(mr, &ml);  // 1)  r = N*l ...        #### MATR.MULTIPLICATION!!!###
        mr.MatrNeg();                         // 2)  r =-N*l
        mr.MatrInc(mb);                       // 3)  r =-N*l+b

        // r = (project_orthogonal(l+diff*r, fric) - l)/diff;
        mr.MatrScale(this->grad_diffstep);
        mr.MatrInc(ml);
        sysd.ConstraintsProject(mr);  // r = P(l+diff*r) ...
        mr.MatrDec(ml);
        mr.MatrScale(1.0 / this->grad_diffstep);  // r = (P(l+diff*r)-l)/diff

        this->tot_iterations++;

        // Terminate iteration when the projected r is small, if (norm(r,2) <= max(rel_tol_b,abs_tol))
        double r_proj_resid = mr.NormTwo();
        if (r_proj_resid < ChMax(rel_tol_b, abs_tol)) {
            if (verbose)
                GetLog() << "Iter=" << iter << " P(r)-converged!  |P(r)|=" << r_proj_resid << "\n";
            break;
        }

        // z_old = z;
        mz_old = mz;

        // z = Mi*r;                                 %% -- Precond
        mz = mr;
        if (do_preconditioning)
            mz.MatrScale(mDi);

        // NMr_old = NMr;
        mNMr_old = mNMr;

        // NMr = N*z;
        sysd.ShurComplementProduct(mNMr, &mz);  // NMr = N*z;    #### MATR.MULTIPLICATION!!!###

        // beta = z'*(NMr-NMr_old)/(z_old'*(NMr_old));
        mtmp.MatrSub(mNMr, mNMr_old);
        double numerator = mz.MatrDot(mz, mtmp);
        double denominator = mz_old.MatrDot(mz_old, mNMr_old);

        double beta = numerator / denominator;

        // Robustness improver: restart if beta=0 or too large
        if (fabs(denominator) < 10e-30 || fabs(numerator) < 10e-30) {
            if (verbose)
                GetLog() << "Iter=" << iter << " Ribiere quotient beta restart: " << numerator << " / " << denominator
                         << "\n";
            beta = 0;
        }

        // beta = ChMax(0.0, beta); //***NOT NEEDED!!! (may be negative in not positive def.matrices!)

        // p = z + beta * p;
        mtmp = mp;
        mtmp.MatrScale(beta);
        mp = mz;
        mp.MatrInc(mtmp);

        // Np = NMr + beta*Np;   // Optimization!! avoid matr x vect!!! (if no 'p' projection has been done)
        mNp.MatrScale(beta);
        mNp.MatrInc(mNMr);

        // ---------------------------------------------
        // METRICS - convergence, plots, etc

        // For recording into correction/residuals/violation history, if debugging
        if (this->record_violation_history)
            AtIterationEnd(r_proj_resid, maxdeltalambda, iter);
    }

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

    return maxviolation;
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

double ChSolverPMINRES::Solve_SupportingStiffness(
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
        GetLog() << "\n-----Projected MINRES -supporting stiffness-, n.vars nx=" << nx
                 << "  max.iters=" << max_iterations << "\n";

    ChMatrixDynamic<> mx(nx, 1);
    ChMatrixDynamic<> md(nx, 1);
    ChMatrixDynamic<> mp(nx, 1);
    ChMatrixDynamic<> mr(nx, 1);
    ChMatrixDynamic<> mz(nx, 1);
    ChMatrixDynamic<> mz_old(nx, 1);
    ChMatrixDynamic<> mZp(nx, 1);
    ChMatrixDynamic<> mMZp(nx, 1);
    ChMatrixDynamic<> mZMr(nx, 1);
    ChMatrixDynamic<> mZMr_old(nx, 1);
    ChMatrixDynamic<> mtmp(nx, 1);
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
        sysd.FromUnknownsToVector(mx);
    else
        mx.FillElem(0);

    // Initialize the d vector filling it with {f, -b}
    sysd.BuildDiVector(md);

    //
    // --- THE P-MINRES ALGORITHM
    //

    double rel_tol = this->rel_tolerance;
    double abs_tol = this->tolerance;
    double rel_tol_d = md.NormInf() * rel_tol;

    // Initial projection of mx   ***TO DO***?
    sysd.UnknownsProject(mx);

    // r = d - Z*x;
    sysd.SystemProduct(
        mr, &mx);    // 1)  r = Z*x ...        #### MATR.MULTIPLICATION!!!### can be avoided if no warm starting!
    mr.MatrNeg();    // 2)  r =-Z*x
    mr.MatrInc(md);  // 3)  r =-Z*x+d
                     /*
                         // r = (project_orthogonal(x+diff*r, fric) - x)/diff;
                         mr.MatrScale(this->grad_diffstep);
                         mr.MatrInc(mx);
                         sysd.UnknownsProject(mr);					// p = P(x+diff*p) ...
                         mr.MatrDec(mx);
                         mr.MatrScale(1.0/this->grad_diffstep);		// p = (P(x+diff*p)-x)/diff
                     */
    // p = Mi * r;
    mp = mr;
    if (do_preconditioning)
        mp.MatrScale(mDi);

    // z = Mi * r;
    mz = mp;

    // ZMr = Z*M*r = Z*z
    sysd.SystemProduct(mZMr, &mz);  // ZMr = Z*z    #### MATR.MULTIPLICATION!!!###

    // Zp = Z*p
    sysd.SystemProduct(mZp, &mp);  // Zp = Z*p    #### MATR.MULTIPLICATION!!!###

    //
    // THE LOOP
    //

    for (int iter = 0; iter < max_iterations; iter++) {
        // MZp = Mi*Zp; % = Mi*Z*p                  %% -- Precond
        mMZp = mZp;
        if (do_preconditioning)
            mMZp.MatrScale(mDi);

        // alpha = (z'*(ZMr))/((MZp)'*(Zp));
        double zZMr = mz.MatrDot(mz, mZMr);      // 1)  zZMr = z'* ZMr
        double MZpZp = mMZp.MatrDot(mMZp, mZp);  // 2)  MZpZp = ((MZp)'*(Zp))

        // Robustness improver: case of division by zero
        if (fabs(MZpZp) < 10e-30) {
            if (verbose)
                GetLog() << "Rayleigh alpha denominator breakdown: " << zZMr << " / " << MZpZp << "=" << (zZMr / MZpZp)
                         << "  iter=" << iter << "\n";
            MZpZp = 10e-30;
        }
        // Robustness improver: case when r is orthogonal to Z*r (ex at first iteration, if f=0, x=0, with constraints)
        if (fabs(zZMr) < 10e-30) {
            if (verbose)
                GetLog() << "Rayleigh alpha numerator breakdown: " << zZMr << " / " << MZpZp << "=" << (zZMr / MZpZp)
                         << "  iter=" << iter << "\n";
            zZMr = 1;
            MZpZp = 1;
        }

        double alpha = zZMr / MZpZp;  // 3)  alpha = (z'*(ZMr))/((MZp)'*(Zp));

        if (alpha < 0)
            if (verbose)
                GetLog() << "Rayleigh alpha < 0: " << alpha << "    iter=" << iter << "\n";

        // x = x + alpha * p;
        mtmp = mp;
        mtmp.MatrScale(alpha);
        mx.MatrInc(mtmp);

        double maxdeltaunknowns = mtmp.NormTwo();  //***better NormInf() for speed reasons?

        // x = Proj(x)
        sysd.UnknownsProject(mx);  // x = P(x)

        // r = d - Z*x;
        sysd.SystemProduct(mr, &mx);  // 1)  r = Z*x ...        #### MATR.MULTIPLICATION!!!###
        mr.MatrNeg();                 // 2)  r =-Z*x
        mr.MatrInc(md);               // 3)  r =-Z*x+d
                                      /*
                                              // r = (project_orthogonal(x+diff*r, fric) - x)/diff;
                                              mr.MatrScale(this->grad_diffstep);
                                              mr.MatrInc(mx);
                                              sysd.UnknownsProject(mr);				// r = P(x+diff*r) ...
                                              mr.MatrDec(mx);
                                              mr.MatrScale(1.0/this->grad_diffstep);	// r = (P(x+diff*r)-x)/diff
                                      */
        this->tot_iterations++;

        // Terminate iteration when the projected r is small, if (norm(r,2) <= max(rel_tol_d,abs_tol))
        double r_proj_resid = mr.NormTwo();
        if (r_proj_resid < ChMax(rel_tol_d, abs_tol)) {
            if (verbose)
                GetLog() << "P(r)-converged! iter=" << iter << " |P(r)|=" << r_proj_resid << "\n";
            break;
        }

        // z_old = z;
        mz_old = mz;

        // z = Mi*r;                                 %% -- Precond
        mz = mr;
        if (do_preconditioning)
            mz.MatrScale(mDi);

        // ZMr_old = ZMr;
        mZMr_old = mZMr;

        // ZMr = Z*z;
        sysd.SystemProduct(mZMr, &mz);  // ZMr = Z*z;    #### MATR.MULTIPLICATION!!!###

        // Ribiere quotient (for flexible preconditioning)
        // beta = z'*(ZMr-ZMr_old)/(z_old'*(ZMr_old));
        mtmp.MatrSub(mZMr, mZMr_old);
        double numerator = mz.MatrDot(mz, mtmp);
        double denominator = mz_old.MatrDot(mz_old, mZMr_old);
        // Rayleigh quotient (original Minres)
        // double numerator   = mr.MatrDot(mz,mZMr);			// 1)  r'* Z *r
        // double denominator = mr.MatrDot(mz_old,mZMr_old);	// 2)  r_old'* Z *r_old
        double beta = numerator / denominator;

        // Robustness improver: restart if beta=0 or too large
        if (fabs(denominator) < 10e-30 || fabs(numerator) < 10e-30) {
            if (verbose)
                GetLog() << "Ribiere quotient beta restart: " << numerator << " / " << denominator << "  iter=" << iter
                         << "\n";
            beta = 0;
        }

        // p = z + beta * p;
        mtmp = mp;
        mtmp.MatrScale(beta);
        mp = mz;
        mp.MatrInc(mtmp);

        // Zp = ZMr + beta*Zp;   // Optimization!! avoid matr x vect!!! (if no 'p' projection has been done)
        mZp.MatrScale(beta);
        mZp.MatrInc(mZMr);

        // ---------------------------------------------
        // METRICS - convergence, plots, etc

        // For recording into correction/residuals/violation history, if debugging
        if (this->record_violation_history)
            AtIterationEnd(r_proj_resid, maxdeltaunknowns, iter);
    }

    // After having solved for unknowns x={q;-l}, now copy those values from x vector to
    // the q values in ChVariable items and to l values in ChConstraint items
    sysd.FromVectorToUnknowns(mx);

    if (verbose)
        GetLog() << "residual: " << mr.NormTwo() << " ---\n";

    return maxviolation;
}

void ChSolverPMINRES::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSolverPMINRES>();
    // serialize parent class
    ChIterativeSolver::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(grad_diffstep);
    marchive << CHNVP(rel_tolerance);
    marchive << CHNVP(diag_preconditioning);
}

void ChSolverPMINRES::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChSolverPMINRES>();
    // deserialize parent class
    ChIterativeSolver::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(grad_diffstep);
    marchive >> CHNVP(rel_tolerance);
    marchive >> CHNVP(diag_preconditioning);
}

}  // end namespace chrono
