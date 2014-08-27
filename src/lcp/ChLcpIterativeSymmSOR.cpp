//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChLcpIterativeSymmSOR.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

  
#include "ChLcpIterativeSymmSOR.h"


namespace chrono 
{

double ChLcpIterativeSymmSOR::Solve(
					ChLcpSystemDescriptor& sysd		///< system description with constraints and variables	   
					)
{ 
	std::vector<ChLcpConstraint*>& mconstraints = sysd.GetConstraintsList();
	std::vector<ChLcpVariables*>&  mvariables	= sysd.GetVariablesList();

	double maxviolation = 0.;
	double maxdeltalambda = 0.;
	int i_friction_comp = 0;
	double old_lambda_friction[3];
	const unsigned int nConstr = (unsigned int) mconstraints.size();
	const unsigned int nVars = (unsigned int) mvariables.size();

	// 1)  Update auxiliary data in all constraints before starting,
	//     that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
	for (unsigned int ic = 0; ic< nConstr; ic++)
		mconstraints[ic]->Update_auxiliary();

	// Average all g_i for the triplet of contact constraints n,u,v.
	//
	int j_friction_comp = 0;
	double gi_values[3];
	for (unsigned int ic = 0; ic< nConstr; ic++)
	{
		if (mconstraints[ic]->GetMode() == CONSTRAINT_FRIC) 
		{
			gi_values[j_friction_comp] = mconstraints[ic]->Get_g_i();
			j_friction_comp++;
			if (j_friction_comp==3)
			{
				double average_g_i = (gi_values[0]+gi_values[1]+gi_values[2])/3.0;
				mconstraints[ic-2]->Set_g_i(average_g_i);
				mconstraints[ic-1]->Set_g_i(average_g_i);
				mconstraints[ic-0]->Set_g_i(average_g_i);
				j_friction_comp=0;
			}
		}	
	}

	// 2)  Compute, for all items with variables, the initial guess for
	//     still uncostrained system:

	for (unsigned int iv = 0; iv< nVars; iv++)
		if (mvariables[iv]->IsActive())
			mvariables[iv]->Compute_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb()); // q = [M]'*fb

	// 3)  For all items with variables, add the effect of initial (guessed)
	//     lagrangian reactions of contraints, if a warm start is desired.
	//     Otherwise, if no warm start, simply resets initial lagrangians to zero.
	if (warm_start)
	{
		for (unsigned int ic = 0; ic< nConstr; ic++)
			if (mconstraints[ic]->IsActive())
				mconstraints[ic]->Increment_q(mconstraints[ic]->Get_l_i());
	}
	else
	{
		for (unsigned int ic = 0; ic< nConstr; ic++)
			mconstraints[ic]->Set_l_i(0.);
	}

	// 4)  Perform the iteration loops
	//
	for (int iter = 0; iter < max_iterations; )
	{
		//
		// Forward sweep, for symmetric SOR
		//

		maxviolation = 0;
		maxdeltalambda = 0;
		i_friction_comp = 0;
		size_t dummy = mconstraints.size();
		for (size_t ic = 0; ic < dummy; ic++)
		{
			// skip computations if constraint not active.
			if (mconstraints[ic]->IsActive())
			{

				// compute residual  c_i = [Cq_i]*q + b_i + cfm_i*l_i
				double mresidual = mconstraints[ic]->Compute_Cq_q() + mconstraints[ic]->Get_b_i()
								 + mconstraints[ic]->Get_cfm_i() * mconstraints[ic]->Get_l_i();

				// true constraint violation may be different from 'mresidual' (ex:clamped if unilateral)
				double candidate_violation = fabs(mconstraints[ic]->Violation(mresidual));

				// compute:  delta_lambda = -(omega/g_i) * ([Cq_i]*q + b_i + cfm_i*l_i )
				double deltal = ( omega / mconstraints[ic]->Get_g_i() ) *
								( -mresidual );

				if (mconstraints[ic]->GetMode() == CONSTRAINT_FRIC)
				{
					candidate_violation = 0;
					// update:   lambda += delta_lambda;
					old_lambda_friction[i_friction_comp] = mconstraints[ic]->Get_l_i();
					mconstraints[ic]->Set_l_i( old_lambda_friction[i_friction_comp]  + deltal);
					i_friction_comp++;

					if (i_friction_comp==1)
						candidate_violation = fabs(ChMin(0.0,mresidual));

					if (i_friction_comp==3)
					{ 
						mconstraints[ic-2]->Project(); // the N normal component will take care of N,U,V

						double new_lambda_0 = mconstraints[ic-2]->Get_l_i() ;
						double new_lambda_1 = mconstraints[ic-1]->Get_l_i() ;
						double new_lambda_2 = mconstraints[ic-0]->Get_l_i() ;
						// Apply the smoothing: lambda= sharpness*lambda_new_projected + (1-sharpness)*lambda_old
						if (this->shlambda!=1.0)
						{
							new_lambda_0 = shlambda*new_lambda_0 + (1.0-shlambda)*old_lambda_friction[0];
							new_lambda_1 = shlambda*new_lambda_1 + (1.0-shlambda)*old_lambda_friction[1];
							new_lambda_2 = shlambda*new_lambda_2 + (1.0-shlambda)*old_lambda_friction[2];
							mconstraints[ic-2]->Set_l_i(new_lambda_0);
							mconstraints[ic-1]->Set_l_i(new_lambda_1);
							mconstraints[ic-0]->Set_l_i(new_lambda_2);
						}
						double true_delta_0 = new_lambda_0 - old_lambda_friction[0];
						double true_delta_1 = new_lambda_1 - old_lambda_friction[1];
						double true_delta_2 = new_lambda_2 - old_lambda_friction[2];
						mconstraints[ic-2]->Increment_q(true_delta_0);
						mconstraints[ic-1]->Increment_q(true_delta_1);
						mconstraints[ic-0]->Increment_q(true_delta_2);
						
						if (this->record_violation_history)
						{
							maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta_0));
							maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta_1));
							maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta_2));
						}
						i_friction_comp =0;
					}
				} 
				else
				{
					// update:   lambda += delta_lambda;
					double old_lambda = mconstraints[ic]->Get_l_i();
					mconstraints[ic]->Set_l_i( old_lambda + deltal);

					// If new lagrangian multiplier does not satisfy inequalities, project
					// it into an admissible orthant (or, in general, onto an admissible set)
					mconstraints[ic]->Project();

					// After projection, the lambda may have changed a bit..
					double new_lambda = mconstraints[ic]->Get_l_i() ;

					// Apply the smoothing: lambda= sharpness*lambda_new_projected + (1-sharpness)*lambda_old
					if (this->shlambda!=1.0)
					{
						new_lambda = shlambda*new_lambda + (1.0-shlambda)*old_lambda;
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
			
			} // end IsActive()
		
		} // end constraint loop 

		// Terminate the loop if violation in constraints has been succesfully limited.
		//if (maxviolation < tolerance)
		//	break;

		// For recording into violation history, if debugging
		if (this->record_violation_history)
			AtIterationEnd(maxviolation, maxdeltalambda, iter);

		// Increment iter count (each sweep, either forward or backward, is considered
		// as a complete iteration, to be fair when comparing to the non-symmetric SOR :)
		iter++;


		//
		// Backward sweep, for symmetric SOR
		//
		maxviolation = 0.;
		maxdeltalambda = 0.;
		i_friction_comp = 0;

		for (int ic = (nConstr-1); ic >= 0 ; ic--)
		{
			// skip computations if constraint not active.
			if (mconstraints[ic]->IsActive())
			{

				// compute residual  c_i = [Cq_i]*q + b_i + cfm_i*l_i
				double mresidual = mconstraints[ic]->Compute_Cq_q() + mconstraints[ic]->Get_b_i()
								 + mconstraints[ic]->Get_cfm_i() * mconstraints[ic]->Get_l_i();

				// true constraint violation may be different from 'mresidual' (ex:clamped if unilateral)
				double candidate_violation = fabs(mconstraints[ic]->Violation(mresidual));

				// compute:  delta_lambda = -(omega/g_i) * ([Cq_i]*q + b_i + cfm_i*l_i )
				double deltal = ( omega / mconstraints[ic]->Get_g_i() ) *
								( -mresidual );

				if (mconstraints[ic]->GetMode() == CONSTRAINT_FRIC)
				{
					candidate_violation = 0;
					// update:   lambda += delta_lambda;
					old_lambda_friction[i_friction_comp] = mconstraints[ic]->Get_l_i();
					mconstraints[ic]->Set_l_i( old_lambda_friction[i_friction_comp]  + deltal);
					i_friction_comp++;
					if (i_friction_comp==3)
					{ 
						mconstraints[ic]->Project(); // the N normal component will take care of N,U,V

						double new_lambda_0 = mconstraints[ic+2]->Get_l_i() ;
						double new_lambda_1 = mconstraints[ic+1]->Get_l_i() ;
						double new_lambda_2 = mconstraints[ic+0]->Get_l_i() ;
						// Apply the smoothing: lambda= sharpness*lambda_new_projected + (1-sharpness)*lambda_old
						if (this->shlambda!=1.0)
						{
							new_lambda_0 = shlambda*new_lambda_0 + (1.0-shlambda)*old_lambda_friction[0];
							new_lambda_1 = shlambda*new_lambda_1 + (1.0-shlambda)*old_lambda_friction[1];
							new_lambda_2 = shlambda*new_lambda_2 + (1.0-shlambda)*old_lambda_friction[2];
							mconstraints[ic-2]->Set_l_i(new_lambda_0);
							mconstraints[ic-1]->Set_l_i(new_lambda_1);
							mconstraints[ic-0]->Set_l_i(new_lambda_2);
						}
						double true_delta_0 = new_lambda_0 - old_lambda_friction[0];
						double true_delta_1 = new_lambda_1 - old_lambda_friction[1];
						double true_delta_2 = new_lambda_2 - old_lambda_friction[2];
						mconstraints[ic+2]->Increment_q(true_delta_0);
						mconstraints[ic+1]->Increment_q(true_delta_1);
						mconstraints[ic+0]->Increment_q(true_delta_2);

						candidate_violation = fabs(ChMin(0.0,mresidual));

						if (this->record_violation_history)
						{
							maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta_0));
							maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta_1));
							maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta_2));
						}
						i_friction_comp =0;
					}
				} 
				else
				{
					// update:   lambda += delta_lambda;
					double old_lambda = mconstraints[ic]->Get_l_i();
					mconstraints[ic]->Set_l_i( old_lambda + deltal);

					// If new lagrangian multiplier does not satisfy inequalities, project
					// it into an admissible orthant (or, in general, onto an admissible set)
					mconstraints[ic]->Project();

					// After projection, the lambda may have changed a bit..
					double new_lambda = mconstraints[ic]->Get_l_i() ;

					// Apply the smoothing: lambda= sharpness*lambda_new_projected + (1-sharpness)*lambda_old
					if (this->shlambda!=1.0)
					{
						new_lambda = shlambda*new_lambda + (1.0-shlambda)*old_lambda;
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
			
			} // end IsActive()
		
		} // end loop on constraints
 
		// For recording into violation history, if debugging
		if (this->record_violation_history)
			AtIterationEnd(maxviolation, maxdeltalambda, iter);

		// Terminate the loop if violation in constraints has been succesfully limited.
		if (maxviolation < tolerance)
			break;	

		iter++;
	}
 
	return maxviolation;

}






} // END_OF_NAMESPACE____


