//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChLcpSolverDEM.cpp
//
//   A solver for DEM type simulations.
//
///////////////////////////////////////////////////


#include "ChLcpSolverDEM.h"


namespace chrono
{


double ChLcpSolverDEM::Solve(ChLcpSystemDescriptor& sysd)
{
	std::vector<ChLcpConstraint*>& mconstraints = sysd.GetConstraintsList();
	std::vector<ChLcpVariables*>&  mvariables   = sysd.GetVariablesList();

	// 1)  Update auxiliary data in all constraints before starting,
	//     that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
	for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
		mconstraints[ic]->Update_auxiliary();

	// 2)  Compute, for all items with variables, the initial guess for
	//     still unconstrained system:
	for (unsigned int iv = 0; iv < mvariables.size(); iv++) {
		if (mvariables[iv]->IsActive())
			mvariables[iv]->Compute_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb()); // q = [M]'*fb
	}

	// 3)  For all items with variables, add the effect of initial (guessed)
	//     lagrangian reactions of contraints, if a warm start is desired.
	//     Otherwise, if no warm start, simply resets initial lagrangians to zero.
	if (warm_start) {
		for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
			if (mconstraints[ic]->IsActive())
				mconstraints[ic]->Increment_q(mconstraints[ic]->Get_l_i());
	} else {
		for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
			mconstraints[ic]->Set_l_i(0.);
	}

	// 4)  Perform the iteration loops (if there are any constraints)
	double maxviolation = 0.;
	double maxdeltalambda = 0.;

	if (mconstraints.size() == 0)
		return maxviolation;

	for (int iter = 0; iter < max_iterations; iter++) {
		maxviolation = 0;
		maxdeltalambda = 0;

		// The iteration on all constraints
		for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
			// skip computations if constraint not active.
			if (mconstraints[ic]->IsActive()) {
				// compute residual  c_i = [Cq_i]*q + b_i
				double mresidual = mconstraints[ic]->Compute_Cq_q() + mconstraints[ic]->Get_b_i();

				// true constraint violation may be different from 'mresidual' (ex:clamped if unilateral)
				double candidate_violation = fabs(mconstraints[ic]->Violation(mresidual));

				// compute:  delta_lambda = -(omega/g_i) * ([Cq_i]*q + b_i )
				double deltal = ( omega / mconstraints[ic]->Get_g_i() ) * ( -mresidual );

				// update:   lambda += delta_lambda;
				double old_lambda = mconstraints[ic]->Get_l_i();
				mconstraints[ic]->Set_l_i( old_lambda + deltal);

				// If new lagrangian multiplier does not satisfy inequalities, project
				// it into an admissible orthant (or, in general, onto an admissible set)
				mconstraints[ic]->Project();

				// After projection, the lambda may have changed a bit..
				double new_lambda = mconstraints[ic]->Get_l_i();

				// Apply the smoothing: lambda= sharpness*lambda_new_projected + (1-sharpness)*lambda_old
				if (this->shlambda != 1.0) {
					new_lambda = shlambda*new_lambda + (1.0-shlambda)*old_lambda;
					mconstraints[ic]->Set_l_i(new_lambda);
				}

				double true_delta = new_lambda - old_lambda;

				// For all items with variables, add the effect of incremented
				// (and projected) lagrangian reactions:
				mconstraints[ic]->Increment_q(true_delta);

				if (this->record_violation_history)
					maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta));

				maxviolation = ChMax(maxviolation, fabs(candidate_violation));

			}  // end IsActive()

		}  // end loop on constraints

		// For recording into violaiton history, if debugging
		if (this->record_violation_history)
			AtIterationEnd(maxviolation, maxdeltalambda, iter);

		// Terminate the loop if violation in constraints has been succesfully limited.
		if (maxviolation < tolerance)
			break;

	}  // end iteration loop

	return maxviolation;
}



} // END_OF_NAMESPACE____


