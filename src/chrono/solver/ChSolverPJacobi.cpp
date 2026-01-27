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

#include "chrono/solver/ChSolverPJacobi.h"
#include "chrono/utils/ChConstants.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverPJacobi)

ChSolverPJacobi::ChSolverPJacobi() : maxviolation(0) {
    m_omega = 0.2;
}

double ChSolverPJacobi::Solve(ChSystemDescriptor& sysd) {
    std::vector<ChConstraint*>& constraints = sysd.GetConstraints();
    std::vector<ChVariables*>& variables = sysd.GetVariables();

    //// TODO
    //// Switch to using Schur complement functions (SchurComplementProduct and SchurComplementRHS) from
    //// ChSystemDescriptor) to accept problems with non-block diagonal mass matrix
    if (sysd.HasMassInverse()) {
        std::cerr
            << "\n\nChSolverPJacobi: Can NOT use PJacobi solver if the system has a non-block diagonal mass matrix\n";
        throw std::runtime_error("ChSolverPJacobi: System descriptor has non-block diagonal mass matrix.");
    }

    m_iterations = 0;
    maxviolation = 0;
    double maxdeltalambda = 0;
    int i_friction_comp = 0;
    double old_lambda_friction[3];

    // 1) Update auxiliary data in all constraints
    //    Average entries for friction constraints
    sysd.SchurComplementUpdateConstraints(true);    

    // 2) Compute, for all items with variables, the initial guess for still unconstrained system: q = M^(-1)*fb
    for (const auto& var : variables) {
        if (var->IsActive())
            var->ComputeMassInverseTimesVector(var->State(), var->Force());
    }

    // 3) For all items with variables, add the effect of initial (guessed)
    //    lagrangian reactions of constraints, if a warm start is desired.
    //    Otherwise, if no warm start, simply resets initial lagrangians to zero.
    if (m_warm_start) {
    } else {
        for (unsigned int ic = 0; ic < constraints.size(); ic++)
            constraints[ic]->SetLagrangeMultiplier(0);
    }

    // 4) Perform the iteration loops
    ChVectorDynamic<> delta_gammas(constraints.size());

    std::fill(violation_history.begin(), violation_history.end(), 0.0);
    std::fill(dlambda_history.begin(), dlambda_history.end(), 0.0);

    for (int iter = 0; iter < m_max_iterations; iter++) {
        // The iteration on all constraints
        //

        maxviolation = 0;
        maxdeltalambda = 0;

        for (unsigned int ic = 0; ic < constraints.size(); ic++) {
            // skip computations if constraint not active.
            if (constraints[ic]->IsActive()) {
                // compute residual  c_i = [Cq_i]*q + b_i + cfm_i*l_i
                double mresidual = constraints[ic]->ComputeJacobianTimesState() +
                                   constraints[ic]->GetRightHandSide() +
                                   constraints[ic]->GetComplianceTerm() * constraints[ic]->GetLagrangeMultiplier();

                // true constraint violation may be different from 'mresidual' (ex:clamped if unilateral)
                double candidate_violation = fabs(constraints[ic]->Violation(mresidual));

                // compute:  delta_lambda = -(omega/g_i) * ([Cq_i]*q + b_i + cfm_i*l_i )
                double deltal = (m_omega / constraints[ic]->GetSchurComplement()) * (-mresidual);

                if (constraints[ic]->GetMode() == ChConstraint::Mode::FRICTION) {
                    candidate_violation = 0;

                    // update:   lambda += delta_lambda;
                    old_lambda_friction[i_friction_comp] = constraints[ic]->GetLagrangeMultiplier();
                    constraints[ic]->SetLagrangeMultiplier(old_lambda_friction[i_friction_comp] + deltal);
                    i_friction_comp++;

                    if (i_friction_comp == 1)
                        candidate_violation = fabs(std::min(0.0, mresidual));

                    if (i_friction_comp == 3) {
                        constraints[ic - 2]->Project();  // the N normal component will take care of N,U,V
                        double new_lambda_0 = constraints[ic - 2]->GetLagrangeMultiplier();
                        double new_lambda_1 = constraints[ic - 1]->GetLagrangeMultiplier();
                        double new_lambda_2 = constraints[ic - 0]->GetLagrangeMultiplier();
                        // Apply the smoothing: lambda= sharpness*lambda_new_projected + (1-sharpness)*lambda_old
                        if (m_shlambda != 1.0) {
                            new_lambda_0 = m_shlambda * new_lambda_0 + (1.0 - m_shlambda) * old_lambda_friction[0];
                            new_lambda_1 = m_shlambda * new_lambda_1 + (1.0 - m_shlambda) * old_lambda_friction[1];
                            new_lambda_2 = m_shlambda * new_lambda_2 + (1.0 - m_shlambda) * old_lambda_friction[2];
                            constraints[ic - 2]->SetLagrangeMultiplier(new_lambda_0);
                            constraints[ic - 1]->SetLagrangeMultiplier(new_lambda_1);
                            constraints[ic - 0]->SetLagrangeMultiplier(new_lambda_2);
                        }
                        delta_gammas[ic - 2] = new_lambda_0 - old_lambda_friction[0];
                        delta_gammas[ic - 1] = new_lambda_1 - old_lambda_friction[1];
                        delta_gammas[ic - 0] = new_lambda_2 - old_lambda_friction[2];
                        // Do NOT update the primal variables now - postponed

                        if (this->record_violation_history) {
                            maxdeltalambda = std::max(maxdeltalambda, fabs(delta_gammas[ic - 2]));
                            maxdeltalambda = std::max(maxdeltalambda, fabs(delta_gammas[ic - 1]));
                            maxdeltalambda = std::max(maxdeltalambda, fabs(delta_gammas[ic - 0]));
                        }
                        i_friction_comp = 0;
                    }
                } else {
                    // update:   lambda += delta_lambda;
                    double old_lambda = constraints[ic]->GetLagrangeMultiplier();
                    constraints[ic]->SetLagrangeMultiplier(old_lambda + deltal);

                    // If new lagrangian multiplier does not satisfy inequalities, project
                    // it into an admissible orthant (or, in general, onto an admissible set)
                    constraints[ic]->Project();

                    // After projection, the lambda may have changed a bit..
                    double new_lambda = constraints[ic]->GetLagrangeMultiplier();

                    // Apply the smoothing: lambda= sharpness*lambda_new_projected + (1-sharpness)*lambda_old
                    if (m_shlambda != 1.0) {
                        new_lambda = m_shlambda * new_lambda + (1.0 - m_shlambda) * old_lambda;
                        constraints[ic]->SetLagrangeMultiplier(new_lambda);
                    }

                    // Do NOT update the primal variables now - postponed

                    delta_gammas[ic] = new_lambda - old_lambda;

                    if (this->record_violation_history)
                        maxdeltalambda = std::max(maxdeltalambda, fabs(delta_gammas[ic]));
                }

                maxviolation = std::max(maxviolation, fabs(candidate_violation));
            }
        }

        // Now, after all deltas are updated, sweep through all constraints and increment  q += [invM][Cq]'* delta_l
        sysd.SchurComplementIncrementVariables(delta_gammas);

        // For recording into violation history, if debugging
        if (this->record_violation_history)
            AtIterationEnd(maxviolation, maxdeltalambda, iter);

        m_iterations++;

        // Terminate the loop if violation in constraints has been successfully limited.
        if (maxviolation < m_tolerance)
            break;
    }

    return maxviolation;
}

}  // end namespace chrono
