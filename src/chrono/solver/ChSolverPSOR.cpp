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

#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/core/ChMathematics.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverPSOR)
CH_UPCASTING(ChSolverPSOR, ChIterativeSolverVI)

ChSolverPSOR::ChSolverPSOR() : maxviolation(0) {}

double ChSolverPSOR::Solve(ChSystemDescriptor& sysd) {
    std::vector<ChConstraint*>& mconstraints = sysd.GetConstraintsList();
    std::vector<ChVariables*>& mvariables = sysd.GetVariablesList();

    m_iterations = 0;
    maxviolation = 0;
    double maxdeltalambda = 0.;
    int i_friction_comp = 0;
    double old_lambda_friction[3];

    // 1)  Update auxiliary data in all constraints before starting,
    //     that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
    for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
        mconstraints[ic]->Update_auxiliary();

    // Average all g_i for the triplet of contact constraints n,u,v.
    //
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

    // 2)  Compute, for all items with variables, the initial guess for
    //     still unconstrained system:

    for (unsigned int iv = 0; iv < mvariables.size(); iv++) {
        if (mvariables[iv]->IsActive())
            mvariables[iv]->Compute_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb());  // q = [M]'*fb
    }

    // 3)  For all items with variables, add the effect of initial (guessed)
    //     lagrangian reactions of constraints, if a warm start is desired.
    //     Otherwise, if no warm start, simply resets initial lagrangians to zero.
    if (m_warm_start) {
        for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
            if (mconstraints[ic]->IsActive())
                mconstraints[ic]->Increment_q(mconstraints[ic]->Get_l_i());
    } else {
        for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
            mconstraints[ic]->Set_l_i(0.);
    }

    // 4)  Perform the iteration loops
    //

    for (int iter = 0; iter < m_max_iterations; iter++) {
        // The iteration on all constraints
        //

        maxviolation = 0;
        maxdeltalambda = 0;
        i_friction_comp = 0;

        for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
            // skip computations if constraint not active.
            if (mconstraints[ic]->IsActive()) {
                // compute residual  c_i = [Cq_i]*q + b_i + cfm_i*l_i
                double mresidual = mconstraints[ic]->Compute_Cq_q() + mconstraints[ic]->Get_b_i() +
                                   mconstraints[ic]->Get_cfm_i() * mconstraints[ic]->Get_l_i();

                // true constraint violation may be different from 'mresidual' (ex:clamped if unilateral)
                double candidate_violation = fabs(mconstraints[ic]->Violation(mresidual));

                // compute:  delta_lambda = -(omega/g_i) * ([Cq_i]*q + b_i + cfm_i*l_i )
                double deltal = (m_omega / mconstraints[ic]->Get_g_i()) * (-mresidual);

                if (mconstraints[ic]->GetMode() == CONSTRAINT_FRIC) {
                    candidate_violation = 0;

                    // update:   lambda += delta_lambda;
                    old_lambda_friction[i_friction_comp] = mconstraints[ic]->Get_l_i();
                    mconstraints[ic]->Set_l_i(old_lambda_friction[i_friction_comp] + deltal);
                    i_friction_comp++;

                    if (i_friction_comp == 1)
                        candidate_violation = fabs(ChMin(0.0, mresidual));

                    if (i_friction_comp == 3) {
                        mconstraints[ic - 2]->Project();  // the N normal component will take care of N,U,V
                        double new_lambda_0 = mconstraints[ic - 2]->Get_l_i();
                        double new_lambda_1 = mconstraints[ic - 1]->Get_l_i();
                        double new_lambda_2 = mconstraints[ic - 0]->Get_l_i();
                        // Apply the smoothing: lambda= sharpness*lambda_new_projected + (1-sharpness)*lambda_old
                        if (m_shlambda != 1.0) {
                            new_lambda_0 = m_shlambda * new_lambda_0 + (1.0 - m_shlambda) * old_lambda_friction[0];
                            new_lambda_1 = m_shlambda * new_lambda_1 + (1.0 - m_shlambda) * old_lambda_friction[1];
                            new_lambda_2 = m_shlambda * new_lambda_2 + (1.0 - m_shlambda) * old_lambda_friction[2];
                            mconstraints[ic - 2]->Set_l_i(new_lambda_0);
                            mconstraints[ic - 1]->Set_l_i(new_lambda_1);
                            mconstraints[ic - 0]->Set_l_i(new_lambda_2);
                        }
                        double true_delta_0 = new_lambda_0 - old_lambda_friction[0];
                        double true_delta_1 = new_lambda_1 - old_lambda_friction[1];
                        double true_delta_2 = new_lambda_2 - old_lambda_friction[2];
                        mconstraints[ic - 2]->Increment_q(true_delta_0);
                        mconstraints[ic - 1]->Increment_q(true_delta_1);
                        mconstraints[ic - 0]->Increment_q(true_delta_2);

                        if (this->record_violation_history) {
                            maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta_0));
                            maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta_1));
                            maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta_2));
                        }
                        i_friction_comp = 0;
                    }
                } else if (mconstraints[ic]->GetMode() == CONSTRAINT_UNILATERAL) {
                    // update:   lambda += delta_lambda;
                    old_lambda_friction[0] = mconstraints[ic]->Get_l_i();
                    mconstraints[ic]->Set_l_i(old_lambda_friction[0] + deltal);

                    candidate_violation = fabs(ChMin(0.0, mresidual));
                    mconstraints[ic]->Project();
                    double new_lambda_0 = mconstraints[ic]->Get_l_i();
                    if (m_shlambda != 1.0) {
                        new_lambda_0 = m_shlambda * new_lambda_0 + (1.0 - m_shlambda) * old_lambda_friction[0];
                        mconstraints[ic]->Set_l_i(new_lambda_0);
                    }

                    double true_delta_0 = new_lambda_0 - old_lambda_friction[0];
                    mconstraints[ic]->Increment_q(true_delta_0);

                    if (this->record_violation_history) {
                        maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta_0));
                    }

                } else {
                    // update:   lambda += delta_lambda;
                    double old_lambda = mconstraints[ic]->Get_l_i();
                    mconstraints[ic]->Set_l_i(old_lambda + deltal);

                    // If new lagrangian multiplier does not satisfy inequalities, project
                    // it into an admissible orthant (or, in general, onto an admissible set)
                    mconstraints[ic]->Project();

                    // After projection, the lambda may have changed a bit..
                    double new_lambda = mconstraints[ic]->Get_l_i();

                    // Apply the smoothing: lambda= sharpness*lambda_new_projected + (1-sharpness)*lambda_old
                    if (m_shlambda != 1.0) {
                        new_lambda = m_shlambda * new_lambda + (1.0 - m_shlambda) * old_lambda;
                        mconstraints[ic]->Set_l_i(new_lambda);
                    }

                    double true_delta = new_lambda - old_lambda;

                    // For all items with variables, add the effect of incremented
                    // (and projected) lagrangian reactions:
                    mconstraints[ic]->Increment_q(true_delta);

                    if (this->record_violation_history)
                        maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta));
                }

                maxviolation = ChMax(maxviolation, fabs(candidate_violation));

            }  // end IsActive()

        }  // end loop on constraints

        // For recording into violation history, if debugging
        if (this->record_violation_history)
            AtIterationEnd(maxviolation, maxdeltalambda, iter);

        m_iterations++;

        // Terminate the loop if violation in constraints has been successfully limited.
        if (maxviolation < m_tolerance)
            break;

    }  // end iteration loop

    return maxviolation;
}

}  // end namespace chrono
