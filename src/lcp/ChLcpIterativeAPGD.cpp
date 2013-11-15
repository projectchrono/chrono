//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChLcpIterativeAPGD.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
  
   
#include "ChLcpIterativeAPGD.h"
#include "ChLcpConstraintTwoFrictionT.h"

namespace chrono
{

double ChLcpIterativeAPGD::Solve(
					ChLcpSystemDescriptor& sysd		///< system description with constraints and variables	
					)
{
	std::vector<ChLcpConstraint*>& mconstraints = sysd.GetConstraintsList();
	std::vector<ChLcpVariables*>&  mvariables	= sysd.GetVariablesList();


	

	double gdiff= 0.000001;

	double maxviolation = 0.;
	int i_friction_comp = 0;

	double theta_k=1.0;
	double theta_k1=theta_k;
	double beta_k1=0.0;

	double L_k=0.0;
	double t_k=0.0;

	tot_iterations = 0;
	// Allocate auxiliary vectors;
	
	int nc = sysd.CountActiveConstraints();
	if (verbose) GetLog() <<"\n-----Accelerated Projected Gradient Descent, solving nc=" << nc << "unknowns \n";

	//ChMatrixDynamic<> ml(nc,1);		//I made this into a class variable so I could print it easier -Hammad
	ml.Resize(nc,1);
	ChMatrixDynamic<> mx(nc,1);
	ChMatrixDynamic<> ms(nc,1);
	ChMatrixDynamic<> my(nc,1);
	ChMatrixDynamic<> ml_candidate(nc,1);
	ChMatrixDynamic<> mg(nc,1);
	ChMatrixDynamic<> mg_tmp(nc,1);
	ChMatrixDynamic<> mg_tmp1(nc,1);
	ChMatrixDynamic<> mg_tmp2(nc,1);
	//ChMatrixDynamic<> mb(nc,1);   //I made this into a class variable so I could print it easier -Hammad
	mb.Resize(nc,1);
	ChMatrixDynamic<> mb_tmp(nc,1);


	// Update auxiliary data in all constraints before starting,
	// that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
		mconstraints[ic]->Update_auxiliary();

	// Average all g_i for the triplet of contact constraints n,u,v.
	//  Can be used for the fixed point phase and/or by preconditioner.
	int j_friction_comp = 0;
	double gi_values[3];
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
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

	// ***TO DO*** move the following thirty lines in a short function ChLcpSystemDescriptor::ShurBvectorCompute() ?

	// Compute the b_shur vector in the Shur complement equation N*l = b_shur
	// with 
	//   N_shur  = D'* (M^-1) * D
	//   b_shur  = - c + D'*(M^-1)*k = b_i + D'*(M^-1)*k
	// but flipping the sign of lambdas,  b_shur = - b_i - D'*(M^-1)*k
	// Do this in three steps:
	
	// Put (M^-1)*k    in  q  sparse vector of each variable..
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
		if (mvariables[iv]->IsActive())
			mvariables[iv]->Compute_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb()); // q = [M]'*fb 

	// ...and now do  b_shur = - D'*q = - D'*(M^-1)*k ..
	int s_i = 0;
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
		if (mconstraints[ic]->IsActive())
		{
			mb(s_i, 0) = - mconstraints[ic]->Compute_Cq_q();
			++s_i;
		}

	// ..and finally do   b_shur = b_shur - c
	sysd.BuildBiVector(mb_tmp);	// b_i   =   -c   = phi/h
	mb.MatrDec(mb_tmp);

		// Optimization: backup the  q  sparse data computed above, 
		// because   (M^-1)*k   will be needed at the end when computing primals.
	ChMatrixDynamic<> mq; 
	sysd.FromVariablesToVector(mq, true);	


	// Initialize lambdas
	if (warm_start)
		sysd.FromConstraintsToVector(ml);
	else
		ml.FillElem(0);


	// Initial projection of ml   ***TO DO***?
	sysd.ConstraintsProject(ml);


	// Fallback solution
	double lastgoodres  = 10e30;
	double lastgoodfval = 10e30;
	ml_candidate.CopyFromMatrix(ml);

	// g = gradient of 0.5*l'*N*l-l'*b
	// g = N*l-b
	sysd.ShurComplementProduct(mg, &ml, 0);		// 1)  g = N*l ...        #### MATR.MULTIPLICATION!!!###
	mg.MatrDec(mb);								// 2)  g = N*l - b_shur ...

	//
	// THE LOOP
	//

	double mf_p =0;

	mb_tmp.FillElem(-1.0);
	mb_tmp.MatrInc(ml);
	sysd.ShurComplementProduct(mg_tmp,&mb_tmp,0); // 1)  g = N*l ...        #### MATR.MULTIPLICATION!!!###
	if(mb_tmp.NormTwo()==0){
		L_k=1;
	}else{
		L_k=mg_tmp.NormTwo()/mb_tmp.NormTwo();
	}
	t_k=1/L_k;

	double obj1=0;
	double obj2=0;

	my.CopyFromMatrix(ml);
	mx.CopyFromMatrix(ml);

	for (int iter = 0; iter < max_iterations; iter++)
	{
		sysd.ShurComplementProduct(mg_tmp1, &my, 0);	// 1)  g_tmp1 = N*yk ...        #### MATR.MULTIPLICATION!!!###
		mg.MatrSub(mg_tmp1,mb);							// 2)  g = N*yk - b_shur ...

		mx.CopyFromMatrix(mg);						// 1) xk1=g
		mx.MatrScale(-t_k); 						// 2) xk1=-tk*g
		mx.MatrInc(my);								// 3) xk1=y-tk*g
		sysd.ConstraintsProject(mx);				// 4) xk1=P(y-tk*g)

		//Now do backtracking for the steplength
		sysd.ShurComplementProduct(mg_tmp, &mx, 0);		// 1)  g_tmp = N*xk1 ...        #### MATR.MULTIPLICATION!!!###
		mg_tmp2.MatrSub(mg_tmp,mb);						// 2)  g_tmp2 = N*xk1 - b_shur ...
		mg_tmp.MatrScale(0.5);							// 3)  g_tmp=0.5*N*xk1
		mg_tmp.MatrDec(mb);								// 4)  g_tmp=0.5*N*xk1-b_shur
		obj1 = mx.MatrDot(&mx,&mg_tmp);					// 5)  obj1=xk1'*(0.5*N*x_k1-b_shur)

		mg_tmp1.MatrScale(0.5);							// 1)  g_tmp1 = 0.5*N*yk
		mg_tmp1.MatrDec(mb);							// 2)  g_tmp1 = 0.5*N*yk-b_shur
		obj2 = my.MatrDot(&my,&mg_tmp1);				// 3)  obj2 = yk'*(0.5*N*yk-b_shur)

		ms.MatrSub(mx,my);								// 1)  s=xk1-yk
		while(obj1>obj2+mg.MatrDot(&mg,&ms)+0.5*L_k*pow(ms.NormTwo(),2.0))
		{
			L_k=2*L_k;
			t_k=1/L_k;

			mx.CopyFromMatrix(mg);						// 1) xk1=g
			mx.MatrScale(-t_k);							// 2) xk1=-tk*g
			mx.MatrInc(my);								// 3) xk1=yk-tk*g
			sysd.ConstraintsProject(mx);				// 4) xk1=P(yk-tk*g)

			sysd.ShurComplementProduct(mg_tmp, &mx, 0);		// 1)  g_tmp = N*xk1 ...        #### MATR.MULTIPLICATION!!!###
			mg_tmp2.MatrSub(mg_tmp,mb);						// 2)  g_tmp2 = N*xk1 - b_shur ...
			mg_tmp.MatrScale(0.5);							// 3)  g_tmp=0.5*N*xk1
			mg_tmp.MatrDec(mb);								// 4)  g_tmp=0.5*N*xk1-b_shur
			obj1 = mx.MatrDot(&mx,&mg_tmp);					// 5)  obj1=xk1'*(0.5*N*x_k1-b_shur)

			ms.MatrSub(mx,my);								// 1)  s=xk1-yk
			if (verbose) GetLog() << "APGD halving stepsize at it " << iter  << "\n";
		}

		theta_k1=(-pow(theta_k,2)+theta_k*sqrt(pow(theta_k,2)+4))/2.0;
		beta_k1=theta_k*(1.0-theta_k)/(pow(theta_k,2)+theta_k1);

		my.CopyFromMatrix(mx);						// 1) y=xk1;
		my.MatrDec(ml);								// 2) y=xk1-xk;
		my.MatrScale(beta_k1);						// 3) y=beta_k1*(xk1-xk);
		my.MatrInc(mx);								// 4) y=xk1+beta_k1*(xk1-xk);
		ms.MatrSub(mx,ml);						    // 0) s = xk1 - xk;

		// Restarting logic if momentum is not appropriate
		if (mg.MatrDot(&mg,&ms)>0)
		{
			my.CopyFromMatrix(mx);					// 1) y=xk1
			theta_k1=1.0;							// 2) theta_k=1
			if (verbose) GetLog() << "Restarting APGD at it " << iter  << "\n";
		}

		//Allow the step to grow...
		L_k=0.9*L_k;
		t_k=1/L_k;

		ml.CopyFromMatrix(mx);						// 1) xk=xk1;
		theta_k=theta_k1;							// 2) theta_k=theta_k1;

		//****METHOD 1 for residual, same as ChLcpIterativeBB
		// Project the gradient (for rollback strategy)
		// g_proj = (l-project_orthogonal(l - gdiff*g, fric))/gdiff;
		mb_tmp.CopyFromMatrix(mg_tmp2);
		mb_tmp.MatrScale(-gdiff);
		mb_tmp.MatrInc(ml);
		sysd.ConstraintsProject(mb_tmp);
		mb_tmp.MatrDec(ml);
		mb_tmp.MatrDivScale(-gdiff);
		double g_proj_norm = mb_tmp.NormTwo(); // NormInf() is faster..
		//****End of METHOD 1 for residual, same as ChLcpIterativeBB

		//****METHOD 2 for residual, same as ChLcpIterativeSOR
		maxviolation = 0;
		i_friction_comp = 0;
		for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
		{
			if (mconstraints[ic]->IsActive())
			{
				// true constraint violation may be different from 'mresidual' (ex:clamped if unilateral)
				double candidate_violation = fabs(mconstraints[ic]->Violation(mg_tmp2.ElementN(ic)));

				if (mconstraints[ic]->GetMode() == CONSTRAINT_FRIC)
				{
					candidate_violation = 0;
					i_friction_comp++;

					if (i_friction_comp==1)
						candidate_violation = fabs(ChMin(0.0,mg_tmp2.ElementN(ic)));

					if (i_friction_comp==3)
						i_friction_comp =0;
				}
				else
				{

				}
				maxviolation = ChMax(maxviolation, fabs(candidate_violation));
			}
		}
		g_proj_norm=maxviolation;
		//****End of METHOD 2 for residual, same as ChLcpIterativeSOR

		// Rollback solution: the last best candidate ('l' with lowest projected gradient)
		// in fact the method is not monotone and it is quite 'noisy', if we do not
		// do this, a prematurely truncated iteration might give a crazy result.
		if(g_proj_norm < lastgoodres)
		{
			lastgoodres  = g_proj_norm;
			ml_candidate = ml;
		}

		// METRICS - convergence, plots, etc

		if (verbose)
		{
			// f_p = 0.5*l_candidate'*N*l_candidate - l_candidate'*b  = l_candidate'*(0.5*Nl_candidate - b);
			sysd.ShurComplementProduct(mg_tmp, &ml_candidate, 0);		// 1)  g_tmp = N*l_candidate ...        #### MATR.MULTIPLICATION!!!###
			mg_tmp.MatrScale(0.5);										// 2)  g_tmp = 0.5*N*l_candidate
			mg_tmp.MatrDec(mb);											// 3)  g_tmp = 0.5*N*l_candidate-b_shur
			mf_p = ml_candidate.MatrDot(&ml_candidate,&mg_tmp);			// 4)  mf_p  = l_candidate'*(0.5*N*l_candidate-b_shur)
		}

		double maxdeltalambda = ms.NormInf();
		double maxd			  = lastgoodres;
			
		// For recording into correction/residuals/violation history, if debugging
		if (this->record_violation_history)
			AtIterationEnd(maxd, maxdeltalambda, iter);

		if (verbose) GetLog() << "  iter=" << iter << "   f=" << mf_p << "  |d|=" << maxd << "  |s|=" << maxdeltalambda  << "\n";

		tot_iterations++;

		// Terminate the loop if violation in constraints has been succesfully limited.
		// ***TO DO*** a reliable termination creterion..
		///*
		if (maxd < this->tolerance)
		{
			if (verbose) GetLog() <<"APGD premature converged at i=" << iter << "\n";
			break;
		}
		//*/

	}

	// Fallback to best found solution (might be useful because of nonmonotonicity)
	ml.CopyFromMatrix(ml_candidate);


	// Resulting DUAL variables:
	// store ml temporary vector into ChLcpConstraint 'l_i' multipliers
	sysd.FromVectorToConstraints(ml); 


	// Resulting PRIMAL variables:
	// compute the primal variables as   v = (M^-1)(k + D*l) 

		// v = (M^-1)*k  ...    (by rewinding to the backup vector computed ad the beginning)
	sysd.FromVectorToVariables(mq);


		// ... + (M^-1)*D*l     (this increment and also stores 'qb' in the ChLcpVariable items)
	for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
	{	
		if (mconstraints[ic]->IsActive())
			mconstraints[ic]->Increment_q( mconstraints[ic]->Get_l_i() );
	}
	

	if (verbose) GetLog() <<"-----\n";
	current_residual = lastgoodres;
	return lastgoodres;

}








} // END_OF_NAMESPACE____


