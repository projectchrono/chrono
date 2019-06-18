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

#include "chrono/solver/ChSolverPCG.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverPCG)

ChSolverPCG::ChSolverPCG(int mmax_iters, bool mwarm_start, double mtolerance)
    : ChIterativeSolver(mmax_iters, mwarm_start, mtolerance, 0.2) {}

double ChSolverPCG::Solve(ChSystemDescriptor& sysd) {
    std::vector<ChConstraint*>& mconstraints = sysd.GetConstraintsList();
    std::vector<ChVariables*>& mvariables = sysd.GetVariablesList();

    tot_iterations = 0;
    double maxviolation = 0.;

    // Update auxiliary data in all constraints before starting,
    // that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        mconstraints[ic]->Update_auxiliary();

    // Allocate auxiliary vectors;

    int nc = sysd.CountActiveConstraints();
    if (verbose)
        GetLog() << "\n-----Projected CG, solving nc=" << nc << "unknowns \n";

    ChVectorDynamic<> ml(nc);
    ChVectorDynamic<> mb(nc);
    ChVectorDynamic<> mu(nc);
    ChVectorDynamic<> mp(nc);
    ChVectorDynamic<> mw(nc);
    ChVectorDynamic<> mz(nc);
    ChVectorDynamic<> mNp(nc);
    ChVectorDynamic<> mtmp(nc);

    double graddiff = 0.00001;  // explorative search step for gradient

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
    mb.setZero();
    int s_i = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        if (mconstraints[ic]->IsActive()) {
            mb(s_i, 0) = -mconstraints[ic]->Compute_Cq_q();
            ++s_i;
        }

    // ..and finally do   b_shur = b_shur - c
    sysd.BuildBiVector(mtmp);  // b_i   =   -c   = phi/h
    mb -= mtmp;

    // Optimization: backup the  q  sparse data computed above,
    // because   (M^-1)*k   will be needed at the end when computing primals.
    ChVectorDynamic<> mq;
    sysd.FromVariablesToVector(mq, true);

    // Initialize lambdas
    if (warm_start)
        sysd.FromConstraintsToVector(ml);
    else
        ml.setZero();

    // Initial projection of ml   ***TO DO***?
    // ...

    std::vector<bool> en_l(nc);
    // Initially all constraints are enabled
    for (int ie = 0; ie < nc; ie++)
        en_l[ie] = true;

    // u = -N*l+b
    sysd.ShurComplementProduct(mu, ml, &en_l);  // u =  N * l
    mu = mb - mu;                               // u = -N * l + b
    mp = mu;

    //
    // THE LOOP
    //

    std::vector<double> f_hist;

    for (int iter = 0; iter < max_iterations; iter++) {
        // alpha =  u'*p / p'*N*p
        sysd.ShurComplementProduct(mNp, mp, &en_l);  // Np = N*p
        double pNp = mp.dot(mNp);                    // pNp = p'*N*p
        double up = mu.dot(mp);                      // up = u'*p
        double alpha = up / pNp;                     // alpha =  u'*p / p'*N*p

        if (fabs(pNp) < 10e-10)
            GetLog() << "Rayleigh quotient pNp breakdown \n";

        // l = l + alpha * p;
        ml += alpha * mp;

        double maxdeltalambda = mtmp.lpNorm<Eigen::Infinity>();

        // l = Proj(l)
        sysd.ConstraintsProject(ml);  // l = P(l)

        // u = -N*l+b
        sysd.ShurComplementProduct(mu, ml, nullptr);  // u = N * l
        mu = mb - mu;                                 // u = -N * l + b

        // w = (Proj(l+lambda*u) -l) /lambda;
        mw = ml + graddiff * mu;
        sysd.ConstraintsProject(mw);  // w = P(l+lambda*u)
        mw = (mw - ml) / graddiff;    // w = (P(l+lambda*u)-l)/lambda

        // z = (Proj(l+lambda*p) -l) /lambda;
        mz = ml + graddiff * mp;
        sysd.ConstraintsProject(mz);  // z = P(l+lambda*u)
        mz = (mz - ml) / graddiff;    // z = (P(l+lambda*u)-l)/lambda

        // beta = w'*Np / pNp;
        double wNp = mw.dot(mNp);
        double beta = wNp / pNp;

        // p = w + beta * z;
        mp = mw + beta * mz;

        // METRICS - convergence, plots, etc
        double maxd = mu.lpNorm<Eigen::Infinity>();  // ***TO DO***  should be max violation, but just for test...

        // For recording into correction/residuals/violation history, if debugging
        if (this->record_violation_history)
            AtIterationEnd(maxd, maxdeltalambda, iter);

        tot_iterations++;
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

}  // end namespace chrono
