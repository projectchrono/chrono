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

#include "chrono/solver/ChSolverAPGD.h"

#include <iostream>
#include <sstream>
#include <string>
#include <valarray>
#include <vector>

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverAPGD)

ChSolverAPGD::ChSolverAPGD() : nc(0), residual(0.0) {}

void ChSolverAPGD::SchurBvectorCompute(ChSystemDescriptor& sysd) {
    // ***TO DO*** move the following thirty lines in a short function ChSystemDescriptor::SchurBvectorCompute() ?

    // Compute the b_schur vector in the Schur complement equation N*l = b_schur
    // with
    //   N_schur  = D'* (M^-1) * D
    //   b_schur  = - c + D'*(M^-1)*k = b_i + D'*(M^-1)*k
    // but flipping the sign of lambdas,  b_schur = - b_i - D'*(M^-1)*k
    // Do this in three steps:

    // Put (M^-1)*k    in  q  sparse vector of each variable..
    for (unsigned int iv = 0; iv < sysd.GetVariables().size(); iv++)
        if (sysd.GetVariables()[iv]->IsActive())
            sysd.GetVariables()[iv]->ComputeMassInverseTimesVector(sysd.GetVariables()[iv]->State(),
                                                                   sysd.GetVariables()[iv]->Force());  // q = [M]'*fb

    // ...and now do  b_schur = - D'*q = - D'*(M^-1)*k ..
    r.setZero();
    int s_i = 0;
    for (unsigned int ic = 0; ic < sysd.GetConstraints().size(); ic++)
        if (sysd.GetConstraints()[ic]->IsActive()) {
            r(s_i, 0) = sysd.GetConstraints()[ic]->ComputeJacobianTimesState();
            ++s_i;
        }

    // ..and finally do   b_schur = b_schur - c
    sysd.BuildBiVector(tmp);  // b_i   =   -c   = phi/h
    r += tmp;
}

double ChSolverAPGD::Res4(ChSystemDescriptor& sysd) {
    // Project the gradient (for rollback strategy)
    // g_proj = (l-project_orthogonal(l - gdiff*g, fric))/gdiff;
    double gdiff = 1.0 / (nc * nc);
    sysd.SchurComplementProduct(tmp, gammaNew);  // tmp = N * gammaNew
    tmp = gammaNew - gdiff * (tmp + r);          // Note: no aliasing issues here
    sysd.ConstraintsProject(tmp);                // tmp = ProjectionOperator(gammaNew - gdiff * g)
    tmp = (gammaNew - tmp) / gdiff;              // Note: no aliasing issues here

    return tmp.norm();
}

double ChSolverAPGD::Solve(ChSystemDescriptor& sysd) {
    bool verbose = false;
    const std::vector<ChConstraint*>& mconstraints = sysd.GetConstraints();
    const std::vector<ChVariables*>& mvariables = sysd.GetVariables();
    if (verbose)
        std::cout << "Number of constraints: " << mconstraints.size()
                  << "\nNumber of variables  : " << mvariables.size() << std::endl;

    // Update auxiliary data in all constraints before starting,
    // that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        mconstraints[ic]->Update_auxiliary();

    double L, t;
    double theta;
    double thetaNew;
    double Beta;
    double obj1, obj2;

    nc = sysd.CountActiveConstraints();
    gamma_hat.resize(nc);
    gammaNew.resize(nc);
    g.resize(nc);
    y.resize(nc);
    gamma.resize(nc);
    yNew.resize(nc);
    r.resize(nc);
    tmp.resize(nc);

    residual = 10e30;

    Beta = 0.0;
    obj1 = 0.0;
    obj2 = 0.0;

    // Compute the b_schur vector in the Schur complement equation N*l = b_schur
    SchurBvectorCompute(sysd);

    // If no constraints, return now. Variables contain M^-1 * f after call to SchurBvectorCompute.
    // This early exit is needed, else we get division by zero and a potential infinite loop.
    if (nc == 0) {
        return 0;
    }

    // Optimization: backup the  q  sparse data computed above,
    // because   (M^-1)*k   will be needed at the end when computing primals.
    ChVectorDynamic<> Minvk;
    sysd.FromVariablesToVector(Minvk, true);

    // (1) gamma_0 = zeros(nc,1)
    if (m_warm_start) {
        for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
            if (mconstraints[ic]->IsActive())
                mconstraints[ic]->IncrementState(mconstraints[ic]->GetLagrangeMultiplier());
    } else {
        for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
            mconstraints[ic]->SetLagrangeMultiplier(0.);
    }
    sysd.FromConstraintsToVector(gamma);

    // (2) gamma_hat_0 = ones(nc,1)
    gamma_hat.setConstant(1.0);

    // (3) y_0 = gamma_0
    y = gamma;

    // (4) theta_0 = 1
    theta = 1.0;

    // (5) L_k = norm(N * (gamma_0 - gamma_hat_0)) / norm(gamma_0 - gamma_hat_0)
    tmp = gamma - gamma_hat;
    L = tmp.norm();
    sysd.SchurComplementProduct(yNew, tmp, nullptr);  // yNew = N * tmp = N * (gamma - gamma_hat)
    L = yNew.norm() / L;
    yNew.setZero();  //// RADU  is this really necessary here?

    // (6) t_k = 1 / L_k
    t = 1.0 / L;

    //// RADU
    //// Check correctness (e.g. sign of 'r' in comments vs. code)

    std::fill(violation_history.begin(), violation_history.end(), 0.0);
    std::fill(dlambda_history.begin(), dlambda_history.end(), 0.0);

    // (7) for k := 0 to N_max
    for (m_iterations = 0; m_iterations < m_max_iterations; m_iterations++) {
        // (8) g = N * y_k - r
        // (9) gamma_(k+1) = ProjectionOperator(y_k - t_k * g)
        sysd.SchurComplementProduct(g, y);  // g = N * y
        gammaNew = y - t * (g + r);
        sysd.ConstraintsProject(gammaNew);

        // (10) while 0.5 * gamma_(k+1)' * N * gamma_(k+1) - gamma_(k+1)' * r >=
        //            0.5 * y_k' * N * y_k - y_k' * r + g' * (gamma_(k+1) - y_k) + 0.5 * L_k * norm(gamma_(k+1) - y_k)^2
        sysd.SchurComplementProduct(tmp, gammaNew);  // tmp = N * gammaNew;
        obj1 = gammaNew.dot(0.5 * tmp + r);

        sysd.SchurComplementProduct(tmp, y);  // tmp = N * y;
        obj2 = y.dot(0.5 * tmp + r) + (gammaNew - y).dot(g + 0.5 * L * (gammaNew - y));

        while (obj1 >= obj2) {
            // (11) L_k = 2 * L_k
            L = 2.0 * L;

            // (12) t_k = 1 / L_k
            t = 1.0 / L;

            // (13) gamma_(k+1) = ProjectionOperator(y_k - t_k * g)
            gammaNew = y - t * g;
            sysd.ConstraintsProject(gammaNew);

            // Update obj1 and obj2
            sysd.SchurComplementProduct(tmp, gammaNew);  // tmp = N * gammaNew;
            obj1 = gammaNew.dot(0.5 * tmp + r);

            sysd.SchurComplementProduct(tmp, y);  // tmp = N * y;
            obj2 = y.dot(0.5 * tmp + r) + (gammaNew - y).dot(g + 0.5 * L * (gammaNew - y));
        }  // (14) endwhile

        // (15) theta_(k+1) = (-theta_k^2 + theta_k * sqrt(theta_k^2 + 4)) / 2
        thetaNew = (-theta * theta + theta * sqrt(theta * theta + 4.0)) / 2.0;

        // (16) Beta_(k+1) = theta_k * (1 - theta_k) / (theta_k^2 + theta_(k+1))
        Beta = theta * (1.0 - theta) / (theta * theta + thetaNew);

        // (17) y_(k+1) = gamma_(k+1) + Beta_(k+1) * (gamma_(k+1) - gamma_k)
        yNew = gammaNew + Beta * (gammaNew - gamma);

        // (18) r = r(gamma_(k+1))
        double res = Res4(sysd);

        if (res < residual) {      // (19) if r < epsilon_min
            residual = res;        // (20) r_min = r
            gamma_hat = gammaNew;  // (21) gamma_hat = gamma_(k+1)
        }                          // (22) endif

        if (residual < m_tolerance) {  // (23) if r < Tau
            break;                     // (24) break
        }                              // (25) endif

        if (g.dot(gammaNew - gamma) > 0) {  // (26) if g' * (gamma_(k+1) - gamma_k) > 0
            yNew = gammaNew;                // (27) y_(k+1) = gamma_(k+1)
            thetaNew = 1.0;                 // (28) theta_(k+1) = 1
        }                                   // (29) endif

        // (30) L_k = 0.9 * L_k
        L = 0.9 * L;

        // (31) t_k = 1 / L_k
        t = 1.0 / L;

        // perform some tasks at the end of the iteration
        if (this->record_violation_history) {
            AtIterationEnd(residual, (gammaNew - gamma).lpNorm<Eigen::Infinity>(), m_iterations);
        }

        // Update iterates
        theta = thetaNew;
        gamma = gammaNew;
        y = yNew;
    }  // (32) endfor

    if (verbose)
        std::cout << "Residual: " << residual << ", Iter: " << m_iterations << std::endl;

    // (33) return Value at time step t_(l+1), gamma_(l+1) := gamma_hat
    sysd.FromVectorToConstraints(gamma_hat);

    // Resulting PRIMAL variables:
    // compute the primal variables as   v = (M^-1)(k + D*l)
    // v = (M^-1)*k  ...    (by rewinding to the backup vector computed at the beginning)
    sysd.FromVectorToVariables(Minvk);

    // ... + (M^-1)*D*l     (this increment and also stores 'qb' in the ChVariable items)
    for (size_t ic = 0; ic < mconstraints.size(); ic++) {
        if (mconstraints[ic]->IsActive())
            mconstraints[ic]->IncrementState(mconstraints[ic]->GetLagrangeMultiplier());
    }

    return residual;
}

void ChSolverAPGD::Dump_Rhs(std::vector<double>& temp) {
    for (int i = 0; i < r.size(); i++) {
        temp.push_back(r(i));
    }
}

void ChSolverAPGD::Dump_Lambda(std::vector<double>& temp) {
    for (int i = 0; i < gamma_hat.size(); i++) {
        temp.push_back(gamma_hat(i));
    }
}

}  // end namespace chrono
