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

ChSolverPMINRES::ChSolverPMINRES()
    : grad_diffstep(0.01),  // too small can cause numerical roundoff troubles!
      rel_tolerance(0.0),
      r_proj_resid(1e30) {}

double ChSolverPMINRES::Solve(ChSystemDescriptor& sysd) {
    std::vector<ChConstraint*>& mconstraints = sysd.GetConstraints();
    std::vector<ChVariables*>& mvariables = sysd.GetVariables();

    // If stiffness blocks are used, the Schur complement cannot be esily
    // used, so fall back to the Solve_SupportingStiffness method, that operates on KKT.
    if (sysd.GetKRMBlocks().size() > 0)
        return this->Solve_SupportingStiffness(sysd);

    // Allocate auxiliary vectors;

    int nc = sysd.CountActiveConstraints();
    if (verbose)
        std::cout << "\n-----Projected MINRES, solving nc=" << nc << "unknowns" << std::endl;

    ChVectorDynamic<> ml(nc);
    ChVectorDynamic<> mb(nc);
    ChVectorDynamic<> mp(nc);
    ChVectorDynamic<> mr(nc);
    ChVectorDynamic<> mz(nc);
    ChVectorDynamic<> mz_old(nc);
    ChVectorDynamic<> mNp(nc);
    ChVectorDynamic<> mMNp(nc);
    ChVectorDynamic<> mNMr(nc);
    ChVectorDynamic<> mNMr_old(nc);
    ChVectorDynamic<> mtmp(nc);
    ChVectorDynamic<> mDi(nc);

    m_iterations = 0;

    // Update auxiliary data in all constraints before starting,
    // that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        mconstraints[ic]->Update_auxiliary();

    // Average all g_i for the triplet of contact constraints n,u,v.
    //  Can be used as diagonal preconditioner.
    int j_friction_comp = 0;
    double gi_values[3];
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
        if (mconstraints[ic]->GetMode() == ChConstraint::Mode::FRICTION) {
            gi_values[j_friction_comp] = mconstraints[ic]->GetSchurComplement();
            j_friction_comp++;
            if (j_friction_comp == 3) {
                double average_g_i = (gi_values[0] + gi_values[1] + gi_values[2]) / 3.0;
                mconstraints[ic - 2]->SetSchurComplement(average_g_i);
                mconstraints[ic - 1]->SetSchurComplement(average_g_i);
                mconstraints[ic - 0]->SetSchurComplement(average_g_i);
                j_friction_comp = 0;
            }
        }
    }

    // The vector with the inverse of diagonal of the N matrix
    mDi.setZero();
    int d_i = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        if (mconstraints[ic]->IsActive()) {
            mDi(d_i, 0) = 1.0 / mconstraints[ic]->GetSchurComplement();
            ++d_i;
        }

    // ***TO DO*** move the following thirty lines in a short function ChSystemDescriptor::SchurBvectorCompute() ?

    // Compute the b_schur vector in the Schur complement equation N*l = b_schur
    // with
    //   N_schur  = D'* (M^-1) * D
    //   b_schur  = - c + D'*(M^-1)*k = b_i + D'*(M^-1)*k
    // but flipping the sign of lambdas,  b_schur = - b_i - D'*(M^-1)*k
    // Do this in three steps:

    // Put (M^-1)*k    in  q  sparse vector of each variable..
    for (unsigned int iv = 0; iv < mvariables.size(); iv++)
        if (mvariables[iv]->IsActive())
            mvariables[iv]->ComputeMassInverseTimesVector(mvariables[iv]->State(),
                                                          mvariables[iv]->Force());  // q = [M]'*fb

    // ...and now do  b_schur = - D' * q  ..
    mb.setZero();
    int s_i = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        if (mconstraints[ic]->IsActive()) {
            mb(s_i, 0) = -mconstraints[ic]->ComputeJacobianTimesState();
            ++s_i;
        }

    // ..and finally do   b_schur = b_schur - c
    sysd.BuildBiVector(mtmp);  // b_i   =   -c   = phi/h
    mb -= mtmp;

    // Optimization: backup the  q  sparse data computed above,
    // because   (M^-1)*k   will be needed at the end when computing primals.
    ChVectorDynamic<> mq;
    sysd.FromVariablesToVector(mq, true);

    double rel_tol = this->rel_tolerance;
    double abs_tol = m_tolerance;
    double rel_tol_b = mb.lpNorm<Eigen::Infinity>() * rel_tol;

    // Initialize lambdas
    if (m_warm_start)
        sysd.FromConstraintsToVector(ml);
    else
        ml.setZero();

    // Initial projection of ml   ***TO DO***?
    // ...

    // r = b - N*l;
    sysd.SchurComplementProduct(mr, ml);  // r = N*l
    mr = mb - mr;                         // r =-N*l+b

    // r = (project_orthogonal(l+diff*r, fric) - l)/diff;
    mr = ml + grad_diffstep * mr;    // r = l + diff*r
    sysd.ConstraintsProject(mr);     // r = P(l + diff*r) ...
    mr = (mr - ml) / grad_diffstep;  // r = (P(l + diff*r) - l)/diff

    // p = Mi * r;
    if (m_use_precond)
        mp = mr.array() * mDi.array();
    else
        mp = mr;

    // z = Mi * r;
    mz = mp;

    // NMr = N*M*r = N*z
    sysd.SchurComplementProduct(mNMr, mz);  // NMr = N*z

    // Np = N*p
    sysd.SchurComplementProduct(mNp, mp);  // Np = N*p

    //// RADU
    //// Is the above correct?  We always have z=p and therefore NMr = Np...

    //
    // THE LOOP
    //

    std::vector<double> f_hist;
    std::fill(violation_history.begin(), violation_history.end(), 0.0);
    std::fill(dlambda_history.begin(), dlambda_history.end(), 0.0);

    for (int iter = 0; iter < m_max_iterations; iter++) {
        // MNp = Mi*Np; % = Mi*N*p                  %% -- Precond
        if (m_use_precond)
            mMNp = mNp.array() * mDi.array();
        else
            mMNp = mNp;

        // alpha = (z'*(NMr))/((MNp)'*(Np));
        double zNMr = mz.dot(mNMr);    // zMNr = z'* NMr
        double MNpNp = mMNp.dot(mNp);  //  MNpNp = ((MNp)'*(Np))

        if (fabs(MNpNp) < 10e-30) {
            if (verbose)
                std::cout << "Iter=" << iter << " Rayleigh quotient alpha breakdown: " << zNMr << " / " << MNpNp
                          << std::endl;
            MNpNp = 10e-12;
        }

        double alpha = zNMr / MNpNp;  // alpha = (z'*(NMr))/((MNp)'*(Np));

        // l = l + alpha * p;
        mtmp = alpha * mp;
        ml += mtmp;

        double maxdeltalambda = mtmp.norm();  //// TODO: better infinity norm for speed reasons?

        // l = Proj(l)
        sysd.ConstraintsProject(ml);  // l = P(l)

        // r = b - N*l;
        sysd.SchurComplementProduct(mr, ml);  // r = N*l
        mr = mb - mr;                         // r =-N*l+b

        // r = (project_orthogonal(l+diff*r, fric) - l)/diff;
        mr = ml + grad_diffstep * mr;
        sysd.ConstraintsProject(mr);     // r = P(l+diff*r)
        mr = (mr - ml) / grad_diffstep;  // r = (P(l+diff*r)-l)/diff

        m_iterations++;

        // Terminate iteration when the projected r is small, if (norm(r,2) <= max(rel_tol_b,abs_tol))
        r_proj_resid = mr.norm();
        if (r_proj_resid < std::max(rel_tol_b, abs_tol)) {
            if (verbose)
                std::cout << "Iter=" << iter << " P(r)-converged!  |P(r)|=" << r_proj_resid << std::endl;
            break;
        }

        // z_old = z;
        mz_old = mz;

        // z = Mi*r;                                 %% -- Precond
        if (m_use_precond)
            mz = mr.array() * mDi.array();
        else
            mz = mr;

        // NMr_old = NMr;
        mNMr_old = mNMr;

        // NMr = N*z;
        sysd.SchurComplementProduct(mNMr, mz);  // NMr = N*z

        // beta = z'*(NMr-NMr_old)/(z_old'*(NMr_old));
        mtmp = mNMr - mNMr_old;
        double numerator = mz.dot(mtmp);
        double denominator = mz_old.dot(mNMr_old);
        double beta = numerator / denominator;

        // Robustness improver: restart if beta=0 or too large
        if (fabs(denominator) < 10e-30 || fabs(numerator) < 10e-30) {
            if (verbose)
                std::cout << "Iter=" << iter << " Ribiere quotient beta restart: " << numerator << " / " << denominator
                          << std::endl;
            beta = 0;
        }

        // beta = std::max(0.0, beta); //// NOT NEEDED (may be negative in not positive def.matrices!)

        // p = z + beta * p;
        mp = mz + beta * mp;

        // Np = NMr + beta*Np;   // Optimization!! avoid matr x vect!!! (if no 'p' projection has been done)
        mNp = mNMr + beta * mNp;

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
            mconstraints[ic]->IncrementState(mconstraints[ic]->GetLagrangeMultiplier());
    }

    if (verbose)
        std::cout << "-----" << std::endl;

    return r_proj_resid;
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

double ChSolverPMINRES::Solve_SupportingStiffness(ChSystemDescriptor& sysd) {
    m_iterations = 0;

    // Allocate auxiliary vectors;

    int nv = sysd.CountActiveVariables();
    int nc = sysd.CountActiveConstraints();
    int nx = nv + nc;  // total scalar unknowns, in x vector for full KKT system Z*x-d=0

    if (verbose)
        std::cout << std::endl
                  << "-----Projected MINRES -supporting stiffness-, n.vars nx=" << nx
                  << "  max.iters=" << m_max_iterations << std::endl;

    ChVectorDynamic<> mx(nx);
    ChVectorDynamic<> md(nx);
    ChVectorDynamic<> mp(nx);
    ChVectorDynamic<> mr(nx);
    ChVectorDynamic<> mz(nx);
    ChVectorDynamic<> mz_old(nx);
    ChVectorDynamic<> mZp(nx);
    ChVectorDynamic<> mMZp(nx);
    ChVectorDynamic<> mZMr(nx);
    ChVectorDynamic<> mZMr_old(nx);
    ChVectorDynamic<> mtmp(nx);
    ChVectorDynamic<> mDi(nx);

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
    if (m_warm_start)
        sysd.FromUnknownsToVector(mx);
    else
        mx.setZero();

    // Initialize the d vector filling it with {f, -b}
    sysd.BuildDiVector(md);

    //
    // --- THE P-MINRES ALGORITHM
    //

    double rel_tol = this->rel_tolerance;
    double abs_tol = m_tolerance;
    double rel_tol_d = md.lpNorm<Eigen::Infinity>() * rel_tol;

    // Initial projection of mx   ***TO DO***?
    sysd.UnknownsProject(mx);

    // r = d - Z*x;
    sysd.SystemProduct(mr, mx);  // r = Z*x
    mr = md - mr;                // r =-Z*x+d

    if (m_use_precond)
        mp = mr.array() * mDi.array();
    else
        mp = mr;

    // z = Mi * r;
    mz = mp;

    // ZMr = Z*M*r = Z*z
    sysd.SystemProduct(mZMr, mz);  // ZMr = Z*z

    // Zp = Z*p
    sysd.SystemProduct(mZp, mp);  // Zp = Z*p

    //
    // THE LOOP
    //

    for (int iter = 0; iter < m_max_iterations; iter++) {
        // MZp = Mi*Zp; % = Mi*Z*p                  %% -- Precond
        if (m_use_precond)
            mMZp = mZp.array() * mDi.array();
        else
            mMZp = mZp;

        // alpha = (z'*(ZMr))/((MZp)'*(Zp));
        double zZMr = mz.dot(mZMr);    // zZMr = z'* ZMr
        double MZpZp = mMZp.dot(mZp);  // MZpZp = ((MZp)'*(Zp))

        // Robustness improver: case of division by zero
        if (fabs(MZpZp) < 10e-30) {
            if (verbose)
                std::cout << "Rayleigh alpha denominator breakdown: " << zZMr << " / " << MZpZp << "=" << (zZMr / MZpZp)
                          << "  iter=" << iter << std::endl;
            MZpZp = 10e-30;
        }

        // Robustness improver: case when r is orthogonal to Z*r (e.g. at first iteration, if f=0, x=0, with
        // constraints)
        if (fabs(zZMr) < 10e-30) {
            if (verbose)
                std::cout << "Rayleigh alpha numerator breakdown: " << zZMr << " / " << MZpZp << "=" << (zZMr / MZpZp)
                          << "  iter=" << iter << std::endl;
            zZMr = 1;
            MZpZp = 1;
        }

        double alpha = zZMr / MZpZp;  // 3)  alpha = (z'*(ZMr))/((MZp)'*(Zp));

        if (alpha < 0)
            if (verbose)
                std::cout << "Rayleigh alpha < 0: " << alpha << "    iter=" << iter << std::endl;

        // x = x + alpha * p;
        mtmp = alpha * mp;
        mx += mtmp;

        double maxdeltaunknowns = mtmp.norm();  //// TODO: better infinity norm for speed reasons?

        // x = Proj(x)
        sysd.UnknownsProject(mx);  // x = P(x)

        // r = d - Z*x;
        sysd.SystemProduct(mr, mx);  // r = Z*x
        mr = md - mr;                // r =-Z*x+d

        m_iterations++;

        // Terminate iteration when the projected r is small, if (norm(r,2) <= max(rel_tol_d,abs_tol))
        r_proj_resid = mr.norm();
        if (r_proj_resid < std::max(rel_tol_d, abs_tol)) {
            if (verbose)
                std::cout << "P(r)-converged! iter=" << iter << " |P(r)|=" << r_proj_resid << std::endl;
            break;
        }

        // z_old = z;
        mz_old = mz;

        // z = Mi*r;                                 %% -- Precond
        if (m_use_precond)
            mz = mr.array() * mDi.array();
        else
            mz = mr;

        // ZMr_old = ZMr;
        mZMr_old = mZMr;

        // ZMr = Z*z;
        sysd.SystemProduct(mZMr, mz);  // ZMr = Z*z

        // Ribiere quotient (for flexible preconditioning)
        //    beta = z'*(ZMr-ZMr_old)/(z_old'*(ZMr_old));
        mtmp = mZMr - mZMr_old;
        double numerator = mz.dot(mtmp);
        double denominator = mz_old.dot(mZMr_old);
        // Rayleigh quotient (original Minres)
        /// double numerator   = mr.MatrDot(mz,mZMr);			// 1)  r'* Z *r
        /// double denominator = mr.MatrDot(mz_old,mZMr_old);	// 2)  r_old'* Z *r_old
        double beta = numerator / denominator;

        // Robustness improver: restart if beta=0 or too large
        if (fabs(denominator) < 10e-30 || fabs(numerator) < 10e-30) {
            if (verbose)
                std::cout << "Ribiere quotient beta restart: " << numerator << " / " << denominator << "  iter=" << iter
                          << std::endl;
            beta = 0;
        }

        // p = z + beta * p;
        mp = mz + beta * mp;

        // Zp = ZMr + beta*Zp;   // Optimization!! avoid matr x vect!!! (if no 'p' projection has been done)
        mZp = mZMr + beta * mZp;

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
        std::cout << "residual: " << mr.norm() << " ---" << std::endl;

    return r_proj_resid;
}

void ChSolverPMINRES::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChSolverPMINRES>();
    // serialize parent class
    ChIterativeSolverVI::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(grad_diffstep);
    archive_out << CHNVP(rel_tolerance);
    archive_out << CHNVP(m_use_precond);
}

void ChSolverPMINRES::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChSolverPMINRES>();
    // deserialize parent class
    ChIterativeSolverVI::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(grad_diffstep);
    archive_in >> CHNVP(rel_tolerance);
    archive_in >> CHNVP(m_use_precond);
}

}  // end namespace chrono
