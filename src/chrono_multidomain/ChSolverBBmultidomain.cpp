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

#include "chrono_multidomain/ChSolverBBmultidomain.h"
#include "chrono_multidomain/ChSystemDescriptorMultidomain.h"
#include "chrono_multidomain/ChDomain.h"
#include "chrono/serialization/ChArchiveBinary.h"

namespace chrono {
namespace multidomain {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverBBmultidomain)
CH_UPCASTING(ChSolverBBmultidomain, ChIterativeSolverVI)

ChSolverBBmultidomain::ChSolverBBmultidomain() : n_armijo(10), max_armijo_backtrace(3), lastgoodres(1e30) {}

double ChSolverBBmultidomain::Solve(ChSystemDescriptor& sysd) {
    std::vector<ChConstraint*>& mconstraints = sysd.GetConstraints();
    std::vector<ChVariables*>& mvariables = sysd.GetVariables();

    ChSystemDescriptorMultidomain& descriptor = dynamic_cast<ChSystemDescriptorMultidomain&>(sysd);

    if (sysd.GetKRMBlocks().size() > 0) {
        std::cerr << "\n\nChSolverBB: Can NOT use Barzilai-Borwein solver if there are stiffness matrices."
                  << std::endl;
        throw std::runtime_error("ChSolverBB: Do NOT use Barzilai-Borwein solver if there are stiffness matrices.");
    }
    
    // MULTIDOMAIN******************
    // This Wv could be needed for the globalVdot() operations at global level.
    // (well, Wv actually NOT used in this algo, in fact here all globalVdot() operate on vectors  
    // that relate to dual variables, ie lagrangian multipliers; since our domain splitting never share any constraint,
    // there are no overlaps between dual vars. This Wv would be needed for clipped vectors of primal variables instead.)
    ChVectorDynamic<>* Wv_partition = &(descriptor.GetDomain()->GetSystem()->CoordWeightsWv());

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
        std::cout << "\n-----Barzilai-Borwein, solving nc=" << nc << "unknowns" << std::endl;

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
    // The vector with the diagonal of the N matrix
    mD.setZero();
    int d_i = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        if (mconstraints[ic]->IsActive()) {
            mD(d_i, 0) = mconstraints[ic]->GetSchurComplement();
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
            mvariables[iv]->ComputeMassInverseTimesVector(mvariables[iv]->State(), mvariables[iv]->Force());  // q = [M]'*fb

    // MULTIDOMAIN******************
    descriptor.SharedVectsToZero(); 
    descriptor.SharedStatesDeltaAddToMultidomainAndSync(1.);
    /*
    // ALTERNATIVE: following stuff is equivalent to descriptor.SharedStates.. line above.
    descriptor.FromVariablesToVector(state);
    descriptor.VectAdditiveToClipped(state);
    descriptor.FromVectorToVariables(state);
    state_old = state;
    */

// UNTIL HERE, IS OK
// THE REST HAS SOME ISSUE TO FIX, RESULT IS INCREASING q CONSTANTLY UP TO EXPLOSION TIME STEP AFTER STEP
// ***TODO*** fix
 assert(false);  // prevent use of solver - not yet usable, work in progress

    // ...and now do  b_schur = - D'*q = - D'*(M^-1)*k ..
    mb.setZero();
    int s_i = 0;
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        if (mconstraints[ic]->IsActive()) {
            mb(s_i, 0) = -mconstraints[ic]->ComputeJacobianTimesState();
            ++s_i;
        }

    // ..and finally do   b_schur = b_schur - c
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
    descriptor.globalSchurComplementProduct(mg, ml);  // 1)  g = N * l   // MULTIDOMAIN******************
    mg -= mb;                             // 2)  g = N * l - b_schur

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
        double dTg = descriptor.globalVdot(mdir,mg); // MULTIDOMAIN******

        // BB dir backward!? fallback to nonpreconditioned dir
        if (dTg > 1e-8) {
            // dir  = [P(l - alpha*g) - l]
            mdir = ml - alpha * mg;         // dir = l - alpha*g
            sysd.ConstraintsProject(mdir);  // dir = P(l - alpha*g) ...
            mdir -= ml;                     // dir = P(l - alpha*g) - l
            // dTg = d'*g;
            dTg = descriptor.globalVdot(mdir,mg); // MULTIDOMAIN******
        }

        double lambda = 1;

        int n_backtracks = 0;
        bool armijo_repeat = true;

        while (armijo_repeat) {
            // l_p = l + lambda*dir;
            ml_p = ml + lambda * mdir;

            // m_tmp = Nl_p = N*l_p;
            descriptor.globalSchurComplementProduct(mb_tmp, ml_p);  // MULTIDOMAIN*****

            // g_p = N * l_p - b  = Nl_p - b
            mg_p = mb_tmp - mb;

            // f_p = 0.5*l_p'*N*l_p - l_p'*b  = l_p'*(0.5*Nl_p - b);
            mf_p = descriptor.globalVdot(ml_p, 0.5 * mb_tmp - mb);    // MULTIDOMAIN*****

            f_hist.push_back(mf_p);

            double max_compare = 10e29;
            for (int h = 1; h <= std::min(iter, this->n_armijo); h++) {
                double compare = f_hist[iter - h] + gamma * lambda * dTg;
                if (compare > max_compare)
                    max_compare = compare;
            }

            if (mf_p > max_compare) {
                armijo_repeat = true;
                if (iter > 0)
                    mf = f_hist[iter - 1];
                double lambdanew = -lambda * lambda * dTg / (2 * (mf_p - mf - lambda * dTg));
                lambda = std::max(sigma_min * lambda, std::min(sigma_max * lambda, lambdanew));
                if (verbose)
                    std::cout << " Repeat Armijo, new lambda=" << lambda << std::endl;
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
            double sDs = descriptor.globalVdot(ms,mb_tmp); // MULTIDOMAIN****
            double sy = descriptor.globalVdot(ms,my); // MULTIDOMAIN****
            if (sy <= 0) {
                alpha = neg_BB1_fallback;
            } else {
                double alph = sDs / sy;  // (s,Ds)/(s,y)   BB1
                alpha = std::min(a_max, std::max(a_min, alph));
            }
        }

        if (((do_BB1e2) && (iter % 2 != 0)) || do_BB2) {
            double sy = descriptor.globalVdot(ms,my); // MULTIDOMAIN****
            if (m_use_precond)
                mb_tmp = my.array() / mD.array();
            else
                mb_tmp = my;
            double yDy = descriptor.globalVdot(my,mb_tmp); // MULTIDOMAIN****
            if (sy <= 0) {
                alpha = neg_BB2_fallback;
            } else {
                double alph = sy / yDy;  // (s,y)/(y,Di*y)   BB2
                alpha = std::min(a_max, std::max(a_min, alph));
            }
        }

        // Project the gradient (for rollback strategy)
        // g_proj = (l-project_orthogonal(l - gdiff*g, fric))/gdiff;
        mb_tmp = ml - gdiff * mg;
        sysd.ConstraintsProject(mb_tmp);     // mb_tmp = ProjectionOperator(l - gdiff * g)
        mb_tmp = (ml - mb_tmp) / gdiff;      // mb_tmp = [l - ProjectionOperator(l - gdiff * g)] / gdiff

        double g_proj_norm = descriptor.globalVnorm(mb_tmp);  // MULTIDOMAIN ****

        // Rollback solution: the last best candidate ('l' with lowest projected gradient)
        // in fact the method is not monotone and it is quite 'noisy', if we do not
        // do this, a prematurely truncated iteration might give a crazy result.
        if (g_proj_norm < lastgoodres) {
            lastgoodres = g_proj_norm;
            ml_candidate = ml;
        }

        // METRICS - convergence, plots, etc

        double maxdeltalambda = descriptor.globalVnorm(ms);  // MULTIDOMAIN ****   as ms.lpNorm<Eigen::Infinity>();
        double maxd = lastgoodres;

        // For recording into correction/residuals/violation history, if debugging
        if (this->record_violation_history)
            AtIterationEnd(maxd, maxdeltalambda, iter);

        if (verbose)
            std::cout << "  iter=" << iter << "   f=" << mf_p << "  |d|=" << maxd << "  |s|=" << maxdeltalambda
                      << std::endl;

        m_iterations++;

        // Terminate the loop if violation in constraints has been successfully limited.
        // ***TO DO*** a reliable termination criterion..
        /*
        if (maxd < m_tolerance)
        {
            std::cout <<"BB premature proj.gradient break at i=" << iter << std::endl;
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
            mconstraints[ic]->IncrementState(mconstraints[ic]->GetLagrangeMultiplier());
    }

    // MULTIDOMAIN******************
    descriptor.SharedStatesDeltaAddToMultidomainAndSync(1.);

    if (verbose)
        std::cout << "-----" << std::endl;

    return lastgoodres;
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

void ChSolverBBmultidomain::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChSolverBBmultidomain>();
    // serialize parent class
    ChIterativeSolverVI::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(n_armijo);
    archive_out << CHNVP(max_armijo_backtrace);
    archive_out << CHNVP(m_use_precond);
}

void ChSolverBBmultidomain::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChSolverBBmultidomain>();
    // deserialize parent class
    ChIterativeSolverVI::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(n_armijo);
    archive_in >> CHNVP(max_armijo_backtrace);
    archive_in >> CHNVP(m_use_precond);
}

}  // end namespace multidomain
}  // end namespace chrono
