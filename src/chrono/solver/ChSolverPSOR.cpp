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
#include "chrono/utils/ChConstants.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverPSOR)
CH_UPCASTING(ChSolverPSOR, ChIterativeSolverVI)

ChSolverPSOR::ChSolverPSOR() : maxviolation(0) {}

double ChSolverPSOR::Solve(ChSystemDescriptor& sysd) {
    std::vector<ChConstraint*>& constraints = sysd.GetConstraints();
    std::vector<ChVariables*>& variables = sysd.GetVariables();

    //// TODO
    //// Switch to using Schur complement functions (SchurComplementProduct and SchurComplementRHS) from
    //// ChSystemDescriptor) to accept problems with non-block diagonal mass matrix
    if (sysd.HasKRMBlocks()) {
        std::cerr << "\n\nChSolverPSOR: Can NOT use PSOR solver if the system includes stiffness or damping matrices"
                  << std::endl;
        throw std::runtime_error("ChSolverPSOR: System descriptor includes stiffness or damping matrices.");
    }

    if (!sysd.SupportsSchurComplement()) {
        std::cerr << "\n\nChSolverPSOR: Can NOT use PSOR solver if\n"
                  << " - there are stiffness or damping matrices, or\n "
                  << " - no inverse mass matrix was provided" << std::endl;
        throw std::runtime_error("ChSolverPSOR: System descriptor does not support Schur complement-based solvers.");
    }

    m_iterations = 0;
    maxviolation = 0;
    double maxdeltalambda = 0.;
    int i_friction_comp = 0;
    double old_lambda_friction[3];

    // 1) Update auxiliary data in all constraints
    //    Average entries for friction constraints
    sysd.SchurComplementUpdateConstraints(true);

    // 2)  Compute, for all items with variables, the initial guess for still unconstrained system: q = M^(-1)*fb
    for (const auto& var : variables) {
        if (var->IsActive())
            var->ComputeMassInverseTimesVector(var->State(), var->Force());
    }

    // 3)  For all items with variables, add the effect of initial (guessed)
    //     lagrangian reactions of constraints, if a warm start is desired.
    //     Otherwise, if no warm start, simply resets initial lagrangians to zero.
    if (m_warm_start) {
        sysd.SchurComplementIncrementVariables();
    } else {
        for (unsigned int ic = 0; ic < constraints.size(); ic++)
            constraints[ic]->SetLagrangeMultiplier(0.);
    }

    // 4)  Perform the iteration loops
    std::fill(violation_history.begin(), violation_history.end(), 0.0);
    std::fill(dlambda_history.begin(), dlambda_history.end(), 0.0);

    for (int iter = 0; iter < m_max_iterations; iter++) {
        maxviolation = 0;
        maxdeltalambda = 0;
        i_friction_comp = 0;

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
                        double true_delta_0 = new_lambda_0 - old_lambda_friction[0];
                        double true_delta_1 = new_lambda_1 - old_lambda_friction[1];
                        double true_delta_2 = new_lambda_2 - old_lambda_friction[2];
                        constraints[ic - 2]->IncrementState(true_delta_0);
                        constraints[ic - 1]->IncrementState(true_delta_1);
                        constraints[ic - 0]->IncrementState(true_delta_2);

                        if (this->record_violation_history) {
                            maxdeltalambda = std::max(maxdeltalambda, fabs(true_delta_0));
                            maxdeltalambda = std::max(maxdeltalambda, fabs(true_delta_1));
                            maxdeltalambda = std::max(maxdeltalambda, fabs(true_delta_2));
                        }
                        i_friction_comp = 0;
                    }
                } else if (constraints[ic]->GetMode() == ChConstraint::Mode::UNILATERAL) {
                    // update:   lambda += delta_lambda;
                    old_lambda_friction[0] = constraints[ic]->GetLagrangeMultiplier();
                    constraints[ic]->SetLagrangeMultiplier(old_lambda_friction[0] + deltal);

                    candidate_violation = fabs(std::min(0.0, mresidual));
                    constraints[ic]->Project();
                    double new_lambda_0 = constraints[ic]->GetLagrangeMultiplier();
                    if (m_shlambda != 1.0) {
                        new_lambda_0 = m_shlambda * new_lambda_0 + (1.0 - m_shlambda) * old_lambda_friction[0];
                        constraints[ic]->SetLagrangeMultiplier(new_lambda_0);
                    }

                    double true_delta_0 = new_lambda_0 - old_lambda_friction[0];
                    constraints[ic]->IncrementState(true_delta_0);

                    if (this->record_violation_history) {
                        maxdeltalambda = std::max(maxdeltalambda, fabs(true_delta_0));
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

                    double true_delta = new_lambda - old_lambda;

                    // For all items with variables, add the effect of incremented
                    // (and projected) lagrangian reactions:
                    constraints[ic]->IncrementState(true_delta);

                    if (this->record_violation_history)
                        maxdeltalambda = std::max(maxdeltalambda, fabs(true_delta));
                }

                maxviolation = std::max(maxviolation, fabs(candidate_violation));

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
