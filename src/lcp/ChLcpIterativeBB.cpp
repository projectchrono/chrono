///////////////////////////////////////////////////
//
//   ChLcpIterativeBB.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
  
   
#include "ChLcpIterativeBB.h"
#include "ChLcpConstraintTwoFrictionT.h"

namespace chrono
{

double ChLcpIterativeBB::Solve(
					ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
					bool add_Mq_to_f 
					)
{
	std::vector<ChLcpConstraint*>& mconstraints = sysd.GetConstraintsList();
	std::vector<ChLcpVariables*>&  mvariables	= sysd.GetVariablesList();

	double maxviolation = 0.;
	int i_friction_comp = 0;
	int iter_tot = 0;

	// Tuning of the spectral gradient search
	double a_min = 10e-10;
	double a_max = 100000.0;
	double sigma_min = 0.1;
	double sigma_max = 0.9;
	double alpha = 0.1;
	double gamma = 1e-4;


	bool verbose = false;

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


	// Allocate auxiliary vectors;
	
	int nc = sysd.CountActiveConstraints();
	if (verbose) GetLog() <<"nc = " << nc << "\n";
	ChMatrixDynamic<> ml(nc,1);
	ChMatrixDynamic<> ml_candidate(nc,1);
	ChMatrixDynamic<> mg(nc,1);
	ChMatrixDynamic<> mg_p(nc,1);
	ChMatrixDynamic<> ml_p(nc,1);
	ChMatrixDynamic<> md(nc,1);
	ChMatrixDynamic<> mb(nc,1);
	ChMatrixDynamic<> mb_tmp(nc,1);
	ChMatrixDynamic<> ms(nc,1);
	ChMatrixDynamic<> my(nc,1);

	double mf;
	double mf_p;



	// Compute the b_shur vector in the Shur complement equation N*l = b_shur
	// with 
	//   N_shur  = D'* (M^-1) * D
	//   b_shur  = - c + D'*(M^-1)*k = b_i + D'*(M^-1)*k
	// but flipping the sign of lambdas,  b_shur = - b_i - D'*(M^-1)*k
	// Do this in three steps:
	
	// Put (M^-1)*k    in  q  sparse vector of each variable..
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
		if (mvariables[iv]->IsActive())
			if (add_Mq_to_f)
				mvariables[iv]->Compute_inc_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb()); // q = q_old + [M]'*fb 
			else
				mvariables[iv]->Compute_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb()); // q = [M]'*fb 

	// ...and now do  b_shur = - D' * q  ..
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

	// Initial projection of ml (to do)
	// ...

	// Fallback solution
	double lastgoodres  = 10e30;
	double lastgoodfval = 10e30;
	ml_candidate = ml;

	// g = gradient of l'*N*l-l'*b 
	// g = N*l-b 
	sysd.ShurComplementProduct(mg, &ml, 0);		// 1)  g = N*l ...        #### MATR.MULTIPLICATION!!!###
	mg.MatrDec(mb);								// 2)  g = N*l - b_shur ...

	mg_p = mg;

	// f = l'*N*l - l'*b  = l'*g
	mf  = ml.MatrDot(&ml,&mg);

	// d  = [P(l - g) - l] 
	md.MatrSub(ml, mg);							// 1) d = (l - g)  ...

	sysd.FromVectorToConstraints(md);	// project 		
	for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
				if (mconstraints[ic]->IsActive())
						mconstraints[ic]->Project();
	sysd.FromConstraintsToVector(md,false);		// 2) d = P(l - g) ...
	
	md.MatrDec(ml);								// 3) d = P(l - g) - l 


	//
	// THE LOOP
	//

	std::vector<double> f_hist;

	for (int iter = 0; iter < max_iterations; iter++)
	{
		// g = N*l - b    = g_p
		mg = mg_p;

		// f = l'*N*l - l'*b  = l'*g
		mf = ml.MatrDot(&ml,&mg);

		// d  = [P(l - alpha*g) - l]
		md.CopyFromMatrix(mg);						// 1) d = g  ...
		md.MatrScale(-alpha);						// 2) d = - alpha*g  ...
		md.MatrInc(ml);								// 3) d = l - alpha*g  ...

		sysd.FromVectorToConstraints(md);	// project 		
		for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
					if (mconstraints[ic]->IsActive())
							mconstraints[ic]->Project();
		sysd.FromConstraintsToVector(md,false);		// 4) d = P(l - alpha*g) ...
		
		md.MatrDec(ml);								// 5) d = P(l - alpha*g) - l


		// dTg = d'*g
		double dTg = md.MatrDot(&md,&mg);

		double lambda = 1;

		f_hist.push_back(mf);
		
		int  n_backtracks = 0;
		bool armijo_repeat = 1;

		while (armijo_repeat)
		{
			// l_p = l + lambda*d;
			ml_p.CopyFromMatrix(md);
			ml_p.MatrScale(lambda);
			ml_p.MatrInc(ml);

			// g_p = N*l_p - b;								#### MATR.MULTIPLICATION!!!###
			sysd.ShurComplementProduct(mg_p, &ml_p, 0);// 1)  g_p = N*l_p ...        
			mg_p.MatrDec(mb);						   // 2)  g_p = N*l_p - b_shur ...

			// f_p = l_p'*g_p;
			mf_p = ml_p.MatrDot(&ml_p,&mg_p);

			double max_compare = 0;
			for (int h = 0; h <= ChMin(iter,this->n_armijo); h++)
			{
				double compare = f_hist[iter-h] + gamma*lambda*dTg;
				if (compare > max_compare)
					max_compare = compare;
			}

			if (mf_p > max_compare)
			{
				armijo_repeat = true;
				double lambdanew = - lambda * lambda * dTg / (2*(mf_p - mf -lambda*dTg));
				lambda = ChMax(sigma_min*lambda, ChMin(sigma_max*lambda,lambdanew));
			}
			else
			{
				armijo_repeat = false;
			}
        
			n_backtracks = n_backtracks +1;
			if (n_backtracks > this->max_armijo_backtrace)
				armijo_repeat = false;
		}

		// s = l_p - l;
		ms.MatrSub(ml_p, ml);
	
		// g_p = N*l_p - b;		not needed?					#### MATR.MULTIPLICATION!!!###
		sysd.ShurComplementProduct(mg_p, &ml_p, 0);  // 1)  g_p = N*l_p ...        
		mg_p.MatrDec(mb);					    	 // 2)  g_p = N*l_p - b_shur ...

		// l = l_p;
		ml.CopyFromMatrix(ml_p);

		// y = -(g_p - g);
		my.MatrSub(mg, mg_p);

		// bk = (s' * y);
		double bk = ms.MatrDot(&ms,&my);

		// Stepsize with Barzilai-Borwein
		if (bk >= 0)
		{
			alpha = a_max;
		}
		else
		{
			double ak = ms.NormTwo(); 
			double alph = -(ak * ak) / bk; 
			alpha = ChMin (a_max, ChMax(a_min, alph));
		}

		// Store sequence of norms of inequality residuals - WAS MATLAB
		/*
		testres = N*l-b;
		resid= norm( min(zeros(nconstr,1),testres) ,2);
		if (resid < lastgoodres)
			lastgoodres = resid;
			l_candidate = l;
			disp('upd');
		end
		res_story(j)= lastgoodres;
		*/

		if (mf_p < lastgoodfval)
		{
			lastgoodfval = mf_p;
			ml_candidate.CopyFromMatrix(ml);
		}

		// METRICS - convergence, plots, etc

		double maxdeltalambda = ms.NormInf();
		double maxd			  = md.NormInf();  // *** should be max violation, but just for test...
			
		// For recording into correction/residuals/violation history, if debugging
		if (this->record_violation_history)
			AtIterationEnd(maxd, maxdeltalambda, iter);

		// Terminate the loop if violation in constraints has been succesfully limited.
		
		/*
		if (maxdeltalambda < this->tolerance)  // ..also check monotonicity of f?
		{
			GetLog() <<"BB premature maxdeltalambda break at i=" << iter << "\n";
			break;
		}
		*/
		
		if (verbose) GetLog() <<"iter="<< iter << " dl=" << maxdeltalambda << " last_f = " <<  lastgoodfval << "\n";

	}

	// Fallback to best found solution (might be useful because of nonmonotonicity)
	ml.CopyFromMatrix(ml_candidate);  // ***TO DO*** ?


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

	return maxviolation;

}








} // END_OF_NAMESPACE____


