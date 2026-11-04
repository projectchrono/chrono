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

#include "chrono/solver/ChSolverLumpedPSOR.h"
#include "chrono/utils/ChConstants.h"
#include <Eigen/Core>
#include <Eigen/Sparse>         
#include <Eigen/Dense>   // only if dense matrix/vector APIs are used here

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverLumpedPSOR)
CH_UPCASTING(ChSolverLumpedPSOR, ChIterativeSolverVI)

ChSolverLumpedPSOR::ChSolverLumpedPSOR() : maxviolation(0) {}


// High performance (dense vector) += (sparse row-major matrix row)*(dense vector) product, 
// avoiding temp objects and avoiding loops on zeroes. Maybe in future Eigen
// will provide an optimal  dense += sparse * dense  to do a more compact
//    q += mat_J.row(row_idx).transpose().cwiseProduct(inv_diag) * factor;
auto add_sparse_row_product = [](Eigen::VectorXd& vec_q, 
        const Eigen::SparseMatrix<double, Eigen::RowMajor>& mat_J, 
        int row_idx, 
        const Eigen::VectorXd& inv_diag, 
        double factor) {
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(mat_J, row_idx); it; ++it) {
        vec_q(it.index()) += it.value() * inv_diag(it.index()) * factor;
    }
};



double ChSolverLumpedPSOR::Solve(ChSystemDescriptor& sysd) {
    std::vector<ChConstraint*>& constraints = sysd.GetConstraints();
    //std::vector<ChVariables*>& variables = sysd.GetVariables();

    //// TODO
    //// Switch to using Schur complement functions (SchurComplementProduct and SchurComplementRHS) from
    //// ChSystemDescriptor) to accept problems with non-block diagonal mass matrix
    //if (sysd.HasKRMBlocks()) {
    //    std::cerr << "\n ChSolverLumpedPSOR: Can NOT use PSOR solver if the system includes stiffness or damping matrices"
    //              << std::endl;
    //    throw std::runtime_error("ChSolverLumpedPSOR: System descriptor includes stiffness or damping matrices.");
    //}
    if (this->diagonal_M.size() != sysd.CountActiveVariables()) {
        std::cerr << "\n ChSolverLumpedPSOR: Can NOT use solver if\n"
                  << " diagonal_M not initialized.\n ";
        throw std::runtime_error("ChSolverLumpedPSOR: No proper diagonal_M is provided.");
    }

    m_iterations = 0;
    maxviolation = 0;
    double maxdeltalambda = 0.;
    int i_friction_comp = 0;
    double old_lambda_friction[3];

    // ***NOTE*** 
    // this version of PSOR uses some system-wide vectors, because it cannot run on per-varaible and per-constraint data
    
    ChVectorDynamic<> inv_diagonal_M = this->diagonal_M.cwiseInverse(); //   [M] ^(-1)   as diagonal 
    ChVectorDynamic<> q(inv_diagonal_M.size());                         //   q, that is the velocity vecotr of all ChVariable

    // 1) Update auxiliary data in all constraints
    
    /////////// 
    // ***NOTE***
    // In conventional PSOR, here we'd call sysd.SchurComplementUpdateConstraints(true); 
    // that computes, at each constraint, its g_i and E_i data. 
    //     g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
    //     [Eq_a]=[invM_a]*[Cq_a]'  etc
    // However...
    // Since we are NOT using the ChVariable own mass blocks, but the diagonal_M instead, we must
    // reimplement what happens in sysd.SchurComplementUpdateConstraints(true), that is
    // reimplement the effect of UpdateAuxiliary() for each constraint, but using the diagonal_M 
    // instead of the ChVariable own mass blocks. 



    // The system level jacobian matrix, row major by default for good performance
    Eigen::SparseMatrix<double, Eigen::RowMajor> J_rows(constraints.size(), inv_diagonal_M.size());
    // Reserve per-row capacities in one call:
    std::vector<Eigen::Index> caps(constraints.size(), 12); // expect an avg of 12 non zeros per jeacobian rows
    J_rows.reserve(caps);


    double g_friction[3];
    unsigned int ic_friction = 0;
    for (unsigned int ic = 0; ic < constraints.size(); ic++) {

        constraints[ic]->PasteJacobianInto(J_rows, ic, 0); 

        double gi = constraints[ic]->GetComplianceTerm();  // include +cfm, consistent with GetSchurComplement meaning

        // Row-major sparse matrix: outer index = row
        for (ChSparseMatrix::InnerIterator it(J_rows, ic); it; ++it) {
            const double cq = it.value();
            gi += cq * cq * inv_diagonal_M[it.col()];
        }

        constraints[ic]->SetSchurComplement(gi);
        if (constraints[ic]->GetMode() == ChConstraint::Mode::FRICTION) {
            g_friction[ic_friction] = gi;
            ic_friction++;
            if (ic_friction == 3) {
                double g_average = (g_friction[0] + g_friction[1] + g_friction[2]) * CH_1_3;
                constraints[ic - 2]->SetSchurComplement(g_average);
                constraints[ic - 1]->SetSchurComplement(g_average);
                constraints[ic - 0]->SetSchurComplement(g_average);
                ic_friction = 0;
            }
        }
    }

    // 2)  Compute, for all items with variables, the initial guess for still unconstrained system: q = M^(-1)*fb
    // ***NOTE***
    // do the equivalent of
    // for (const auto& var : variables) 
    //    if (var->IsActive())
    //        var->ComputeMassInverseTimesVector(var->State(), var->Force());
    ChVectorDynamic<> Fb(inv_diagonal_M.size());
    sysd.BuildFbVector(Fb);  

    q = inv_diagonal_M.cwiseProduct(Fb); //  q =  M ^ (-1) * fb
    

    // 3)  For all items with variables, add the effect of initial (guessed)
    //     lagrangian reactions of constraints, if a warm start is desired.
    //     Otherwise, if no warm start, simply resets initial lagrangians to zero.
    if (m_warm_start) {
        // ***NOTE***
        // do the equivalent of sysd.SchurComplementIncrementVariables();  but with lumped masses:
        // q += M ^ (-1) * Cq' * l
        for (unsigned int ic = 0; ic < constraints.size(); ic++) {
            if (constraints[ic]->IsActive())
                add_sparse_row_product(q, J_rows, ic, inv_diagonal_M, constraints[ic]->GetLagrangeMultiplier()); 
                // or .. q += J_rows.row(ic).transpose().cwiseProduct(inv_diagonal_M) * constraints[ic]->GetLagrangeMultiplier();
        }
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
                double mresidual = J_rows.row(ic).dot(q) +  // ***NOTE*** equivalent to:  constraints[ic]->ComputeJacobianTimesState() +
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
                        // ***NOTE*** do the equivalent of:
                        //constraints[ic - 2]->IncrementState(true_delta_0);
                        //constraints[ic - 1]->IncrementState(true_delta_1);
                        //constraints[ic - 0]->IncrementState(true_delta_2);   or...
                        //q += J_rows.row(ic - 2).transpose().cwiseProduct(inv_diagonal_M) * true_delta_0;
                        //q += J_rows.row(ic - 1).transpose().cwiseProduct(inv_diagonal_M) * true_delta_1;
                        //q += J_rows.row(ic - 0).transpose().cwiseProduct(inv_diagonal_M) * true_delta_2;
                        add_sparse_row_product(q, J_rows, (ic-2), inv_diagonal_M, true_delta_0);
                        add_sparse_row_product(q, J_rows, (ic-1), inv_diagonal_M, true_delta_1);
                        add_sparse_row_product(q, J_rows, (ic-0), inv_diagonal_M, true_delta_2);

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
                    
                    // ***NOTE*** do the equivalent of:
                    // constraints[ic]->IncrementState(true_delta_0);  or...
                    //q += J_rows.row(ic).transpose().cwiseProduct(inv_diagonal_M) * true_delta_0; 
                    add_sparse_row_product(q, J_rows, ic, inv_diagonal_M, true_delta_0);

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

                    // ***NOTE*** do the equivalent of:
                    // constraints[ic]->IncrementState(true_delta);   or... 
                    //q += J_rows.row(ic).transpose().cwiseProduct(inv_diagonal_M) * true_delta;
                    add_sparse_row_product(q, J_rows, ic, inv_diagonal_M, true_delta);

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


    // ***NOTE*** from system wide vector to variables
    sysd.FromVectorToVariables(q);

    return maxviolation;
}

}  // end namespace chrono
