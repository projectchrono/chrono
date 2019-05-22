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

#include "chrono/core/ChStream.h"

#include <iostream>
#include <sstream>
#include <string>
#include <valarray>
#include <vector>

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverAPGD)

void ChSolverAPGD::ShurBvectorCompute(ChSystemDescriptor& sysd) {
    // ***TO DO*** move the following thirty lines in a short function ChSystemDescriptor::ShurBvectorCompute() ?

    // Compute the b_shur vector in the Shur complement equation N*l = b_shur
    // with
    //   N_shur  = D'* (M^-1) * D
    //   b_shur  = - c + D'*(M^-1)*k = b_i + D'*(M^-1)*k
    // but flipping the sign of lambdas,  b_shur = - b_i - D'*(M^-1)*k
    // Do this in three steps:

    // Put (M^-1)*k    in  q  sparse vector of each variable..
    for (unsigned int iv = 0; iv < sysd.GetVariablesList().size(); iv++)
        if (sysd.GetVariablesList()[iv]->IsActive())
            sysd.GetVariablesList()[iv]->Compute_invMb_v(sysd.GetVariablesList()[iv]->Get_qb(),
                                                         sysd.GetVariablesList()[iv]->Get_fb());  // q = [M]'*fb

    // ...and now do  b_shur = - D'*q = - D'*(M^-1)*k ..
    r.Reset();
    int s_i = 0;
    for (unsigned int ic = 0; ic < sysd.GetConstraintsList().size(); ic++)
        if (sysd.GetConstraintsList()[ic]->IsActive()) {
            r(s_i, 0) = sysd.GetConstraintsList()[ic]->Compute_Cq_q();
            ++s_i;
        }

    // ..and finally do   b_shur = b_shur - c
    sysd.BuildBiVector(tmp);  // b_i   =   -c   = phi/h
    r.MatrInc(tmp);
}

double ChSolverAPGD::Res4(ChSystemDescriptor& sysd) {
    // Project the gradient (for rollback strategy)
    // g_proj = (l-project_orthogonal(l - gdiff*g, fric))/gdiff;
    double gdiff = 1.0 / pow(nc, 2.0);
    sysd.ShurComplementProduct(tmp, &gammaNew);
    tmp.MatrInc(r);
    tmp.MatrScale(-gdiff);
    tmp.MatrInc(gammaNew);
    sysd.ConstraintsProject(tmp);
    tmp.MatrSub(gammaNew, tmp);
    tmp.MatrScale(1.0 / gdiff);

    return tmp.NormTwo();
}

double ChSolverAPGD::Solve(ChSystemDescriptor& sysd) {
    bool verbose = false;
    const std::vector<ChConstraint*>& mconstraints = sysd.GetConstraintsList();
    const std::vector<ChVariables*>& mvariables = sysd.GetVariablesList();
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
    gamma_hat.Resize(nc, 1);
    gammaNew.Resize(nc, 1);
    g.Resize(nc, 1);
    y.Resize(nc, 1);
    gamma.Resize(nc, 1);
    yNew.Resize(nc, 1);
    r.Resize(nc, 1);
    tmp.Resize(nc, 1);

    residual = 10e30;

    Beta = 0.0;
    obj1 = 0.0;
    obj2 = 0.0;

    // Compute the b_shur vector in the Shur complement equation N*l = b_shur
    ShurBvectorCompute(sysd);

    // Optimization: backup the  q  sparse data computed above,
    // because   (M^-1)*k   will be needed at the end when computing primals.
    ChMatrixDynamic<> Minvk;
    sysd.FromVariablesToVector(Minvk, true);

    // (1) gamma_0 = zeros(nc,1)
    if (warm_start) {
        for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
            if (mconstraints[ic]->IsActive())
                mconstraints[ic]->Increment_q(mconstraints[ic]->Get_l_i());
    } else {
        for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
            mconstraints[ic]->Set_l_i(0.);
    }
    sysd.FromConstraintsToVector(gamma);

    // (2) gamma_hat_0 = ones(nc,1)
    gamma_hat.FillElem(1);

    // (3) y_0 = gamma_0
    y.CopyFromMatrix(gamma);

    // (4) theta_0 = 1
    theta = 1.0;

    // (5) L_k = norm(N * (gamma_0 - gamma_hat_0)) / norm(gamma_0 - gamma_hat_0)
    tmp.MatrSub(gamma, gamma_hat);
    L = tmp.NormTwo();
    sysd.ShurComplementProduct(yNew, &tmp, 0);
    L = yNew.NormTwo() / L;
    yNew.FillElem(0);  // reset yNew to be all zeros

    // (6) t_k = 1 / L_k
    t = 1.0 / L;

    // (7) for k := 0 to N_max
    for (tot_iterations = 0; tot_iterations < max_iterations; tot_iterations++) {
        // (8) g = N * y_k - r
        sysd.ShurComplementProduct(g, &y);
        g.MatrInc(r);

        // (9) gamma_(k+1) = ProjectionOperator(y_k - t_k * g)
        gammaNew.CopyFromMatrix(g);
        gammaNew.MatrScale(-t);
        gammaNew.MatrInc(y);
        sysd.ConstraintsProject(gammaNew);

        // (10) while 0.5 * gamma_(k+1)' * N * gamma_(k+1) - gamma_(k+1)' * r >= 0.5 * y_k' * N * y_k - y_k' * r + g' *
        // (gamma_(k+1) - y_k) + 0.5 * L_k * norm(gamma_(k+1) - y_k)^2
        sysd.ShurComplementProduct(tmp, &gammaNew);  // Here tmp is equal to N*gammaNew;
        tmp.MatrScale(0.5);
        tmp.MatrInc(r);
        obj1 = tmp.MatrDot(gammaNew, tmp);

        sysd.ShurComplementProduct(tmp, &y);  // Here tmp is equal to N*y;
        tmp.MatrScale(0.5);
        tmp.MatrInc(r);
        obj2 = tmp.MatrDot(y, tmp);
        tmp.MatrSub(gammaNew, y);  // Here tmp is equal to gammaNew - y
        obj2 = obj2 + tmp.MatrDot(tmp, g) + 0.5 * L * tmp.MatrDot(tmp, tmp);

        while (obj1 >= obj2) {
            // (11) L_k = 2 * L_k
            L = 2.0 * L;

            // (12) t_k = 1 / L_k
            t = 1.0 / L;

            // (13) gamma_(k+1) = ProjectionOperator(y_k - t_k * g)
            gammaNew.CopyFromMatrix(g);
            gammaNew.MatrScale(-t);
            gammaNew.MatrInc(y);
            sysd.ConstraintsProject(gammaNew);

            // Update the components of the while condition
            sysd.ShurComplementProduct(tmp, &gammaNew);  // Here tmp is equal to N*gammaNew;
            tmp.MatrScale(0.5);
            tmp.MatrInc(r);
            obj1 = tmp.MatrDot(gammaNew, tmp);

            sysd.ShurComplementProduct(tmp, &y);  // Here tmp is equal to N*y;
            tmp.MatrScale(0.5);
            tmp.MatrInc(r);
            obj2 = tmp.MatrDot(y, tmp);
            tmp.MatrSub(gammaNew, y);  // Here tmp is equal to gammaNew - y
            obj2 = obj2 + tmp.MatrDot(tmp, g) + 0.5 * L * tmp.MatrDot(tmp, tmp);

            // (14) endwhile
        }

        // (15) theta_(k+1) = (-theta_k^2 + theta_k * sqrt(theta_k^2 + 4)) / 2
        thetaNew = (-pow(theta, 2.0) + theta * sqrt(pow(theta, 2.0) + 4.0)) / 2.0;

        // (16) Beta_(k+1) = theta_k * (1 - theta_k) / (theta_k^2 + theta_(k+1))
        Beta = theta * (1.0 - theta) / (pow(theta, 2) + thetaNew);

        // (17) y_(k+1) = gamma_(k+1) + Beta_(k+1) * (gamma_(k+1) - gamma_k)
        tmp.MatrSub(gammaNew, gamma);  // Here tmp is equal to gammaNew - gamma;
        tmp.MatrScale(Beta);
        yNew.MatrAdd(gammaNew, tmp);

        // (18) r = r(gamma_(k+1))
        double res = Res4(sysd);

        // (19) if r < epsilon_min
        if (res < residual) {
            // (20) r_min = r
            residual = res;

            // (21) gamma_hat = gamma_(k+1)
            gamma_hat.CopyFromMatrix(gammaNew);

            // (22) endif
        }

        // (23) if r < Tau
        if (residual < this->tolerance) {
            // (24) break
            break;

            // (25) endif
        }

        // (26) if g' * (gamma_(k+1) - gamma_k) > 0
        tmp.MatrSub(gammaNew, gamma);
        if (tmp.MatrDot(tmp, g) > 0) {
            // (27) y_(k+1) = gamma_(k+1)
            yNew.CopyFromMatrix(gammaNew);

            // (28) theta_(k+1) = 1
            thetaNew = 1.0;

            // (29) endif
        }

        // (30) L_k = 0.9 * L_k
        L = 0.9 * L;

        // (31) t_k = 1 / L_k
        t = 1.0 / L;

        // perform some tasks at the end of the iteration
        if (this->record_violation_history) {
            tmp.MatrSub(gammaNew, gamma);
            AtIterationEnd(residual, tmp.NormInf(), tot_iterations);
        }

        // Update iterates
        theta = thetaNew;
        gamma.CopyFromMatrix(gammaNew);
        y.CopyFromMatrix(yNew);

        // (32) endfor
    }

    if (verbose)
        std::cout << "Residual: " << residual << ", Iter: " << tot_iterations << std::endl;

    // (33) return Value at time step t_(l+1), gamma_(l+1) := gamma_hat
    sysd.FromVectorToConstraints(gamma_hat);

    // Resulting PRIMAL variables:
    // compute the primal variables as   v = (M^-1)(k + D*l)
    // v = (M^-1)*k  ...    (by rewinding to the backup vector computed at the beginning)
    sysd.FromVectorToVariables(Minvk);

    // ... + (M^-1)*D*l     (this increment and also stores 'qb' in the ChVariable items)
    for (size_t ic = 0; ic < mconstraints.size(); ic++) {
        if (mconstraints[ic]->IsActive())
            mconstraints[ic]->Increment_q(mconstraints[ic]->Get_l_i());
    }

    return residual;
}

}  // end namespace chrono
