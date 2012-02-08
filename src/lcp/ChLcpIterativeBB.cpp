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


	// Tuning of the spectral gradient search
	double a_min = 1e-13;
	double a_max = 1e13;
	double sigma_min = 0.1;
	double sigma_max = 0.9;
	double alpha = 0.0001;
	double gamma = 1e-4;
	double gdiff= 0.000001;
	bool do_preconditioning = true;

	bool do_BB1e2= true;
	bool do_BB1	 = false;
	bool do_BB2	 = false;
	double neg_BB1_fallback = 0.11;
	double neg_BB2_fallback = 0.12;

	bool verbose = false;


	int i_friction_comp = 0;
	int iter_tot = 0;

	// Allocate auxiliary vectors;
	
	int nc = sysd.CountActiveConstraints();
	if (verbose) GetLog() <<"\n-----Barzilai-Borwein, solving nc=" << nc << "unknowns \n";

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
	ChMatrixDynamic<> mD (nc,1);
	ChMatrixDynamic<> mDg (nc,1);


	
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
	// The vector with the diagonal of the N matrix
	int d_i = 0;
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
		if (mconstraints[ic]->IsActive())
		{
			mD(d_i, 0) = mconstraints[ic]->Get_g_i();
			++d_i;
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
			if (add_Mq_to_f)
				mvariables[iv]->Compute_inc_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb()); // q = q_old + [M]'*fb 
			else
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
	ml_candidate = ml;

	// g = gradient of 0.5*l'*N*l-l'*b 
	// g = N*l-b 
	sysd.ShurComplementProduct(mg, &ml, 0);		// 1)  g = N*l ...        #### MATR.MULTIPLICATION!!!###
	mg.MatrDec(mb);								// 2)  g = N*l - b_shur ...

	mg_p = mg;


	//
	// THE LOOP
	//

	double mf_p =0;
	double mf =1e29;
	std::vector<double> f_hist;

	for (int iter = 0; iter < max_iterations; iter++)
	{
		// Dg = Di*g;
		mDg = mg;
		if (do_preconditioning)
			mDg.MatrDivScale(mD);
		
		// d  = [P(l - alpha*Dg) - l]
		md.CopyFromMatrix(mDg);						// 1) d = Dg  ...
		md.MatrScale(-alpha);						// 2) d = - alpha*Dg  ...
		md.MatrInc(ml);								// 3) d = l - alpha*Dg  ...
		sysd.ConstraintsProject(md);				// 4) d = P(l - alpha*Dg) ...
		md.MatrDec(ml);								// 5) d = P(l - alpha*Dg) - l

		// dTg = d'*g;
		double dTg = md.MatrDot(&md,&mg);

		// BB dir backward!? fallback to nonpreconditioned dir
		if (dTg > 1e-8)
		{	
			// d  = [P(l - alpha*g) - l]
			md.CopyFromMatrix(mg);						// 1) d = g  ...
			md.MatrScale(-alpha);						// 2) d = - alpha*g  ...
			md.MatrInc(ml);								// 3) d = l - alpha*g  ...
			sysd.ConstraintsProject(md);				// 4) d = P(l - alpha*g) ...
			md.MatrDec(ml);								// 5) d = P(l - alpha*g) - l
			// dTg = d'*g;
			dTg = md.MatrDot(&md,&mg);
		}

		double lambda = 1;

		
		
		int  n_backtracks = 0;
		bool armijo_repeat = true;

		while (armijo_repeat)
		{
			// l_p = l + lambda*d;
			ml_p.CopyFromMatrix(md);
			ml_p.MatrScale(lambda);
			ml_p.MatrInc(ml);

			// Nl_p = N*l_p;                        #### MATR.MULTIPLICATION!!!###
			sysd.ShurComplementProduct(mb_tmp, &ml_p, 0);// 1)  mb_tmp = N*l_p  = Nl_p 

			// g_p = N*l_p - b  = Nl_p - b		
			mg_p.MatrSub(mb_tmp, mb);					 // 2)  g_p = N*l_p - b

			// f_p = 0.5*l_p'*N*l_p - l_p'*b  = l_p'*(0.5*Nl_p - b);
			mb_tmp.MatrScale(0.5);
			mb_tmp.MatrDec(mb);
			mf_p = ml_p.MatrDot(&ml_p,&mb_tmp);
			
			f_hist.push_back(mf_p);


			double max_compare = 10e29;
			for (int h = 1; h <= ChMin(iter,this->n_armijo); h++)
			{
				double compare = f_hist[iter-h] + gamma*lambda*dTg;
				if (compare > max_compare)
					max_compare = compare;
			}

			if (mf_p > max_compare)
			{
				armijo_repeat = true;
				if (iter>0)
					mf = f_hist[iter-1];
				double lambdanew = - lambda * lambda * dTg / (2*(mf_p - mf -lambda*dTg));
				lambda = ChMax(sigma_min*lambda, ChMin(sigma_max*lambda,lambdanew));
				if (verbose)  GetLog() << " Repeat Armijo, new lambda=" << lambda << "\n";
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
		
		// y = g_p - g;
		my.MatrSub(mg_p, mg);

		// l = l_p;
		ml.CopyFromMatrix(ml_p);

		// g = g_p;
		mg.CopyFromMatrix(mg_p);

		if (((do_BB1e2) && (iter%2 ==0)) || do_BB1)
		{
			mb_tmp = ms;
			if (do_preconditioning)
				mb_tmp.MatrScale(mD);
			double sDs = ms.MatrDot(&ms,&mb_tmp);
			double sy  = ms.MatrDot(&ms, &my);
			if (sy <= 0)
			{
				alpha = neg_BB1_fallback;
			}
			else
			{
				double alph = sDs / sy;  // (s,Ds)/(s,y)   BB1
				alpha = ChMin (a_max, ChMax(a_min, alph));
			}
		}

		/*
		// this is a modified rayleight quotient - looks like it works anyway...
		if (((do_BB1e2) && (iter%2 ==0)) || do_BB1)
		{
			double ss = ms.MatrDot(&ms,&ms);
			mb_tmp = my;
			if (do_preconditioning)
				mb_tmp.MatrDivScale(mD);
			double sDy = ms.MatrDot(&ms, &mb_tmp);
			if (sDy <= 0)
			{
				alpha = neg_BB1_fallback;
			}
			else
			{
				double alph = ss / sDy;  // (s,s)/(s,Di*y)   BB1 (modified version)
				alpha = ChMin (a_max, ChMax(a_min, alph));
			}
		}
		*/

		if (((do_BB1e2) && (iter%2 !=0)) || do_BB2)
		{
			double sy = ms.MatrDot(&ms,&my);
			mb_tmp = my;
			if (do_preconditioning)
				mb_tmp.MatrDivScale(mD);
			double yDy = my.MatrDot(&my, &mb_tmp);
			if (sy <= 0)
			{
				alpha = neg_BB2_fallback;
			}
			else
			{
				double alph = sy / yDy;  // (s,y)/(y,Di*y)   BB2
				alpha = ChMin (a_max, ChMax(a_min, alph));
			}
		}

        // Project the gradient (for rollback strategy)
		// g_proj = (l-project_orthogonal(l - gdiff*g, fric))/gdiff;
		mb_tmp = mg;
		mb_tmp.MatrScale(-gdiff);
		mb_tmp.MatrInc(ml);
		sysd.ConstraintsProject(mb_tmp);
		mb_tmp.MatrDec(ml);
		mb_tmp.MatrDivScale(-gdiff);

		double g_proj_norm = mb_tmp.NormTwo(); // NormInf() is faster..

		// Rollback solution: the last best candidate ('l' with lowest projected gradient)
		// in fact the method is not monotone and it is quite 'noisy', if we do not
		// do this, a prematurely truncated iteration might give a crazy result.
        if(g_proj_norm < lastgoodres)
		{
            lastgoodres  = g_proj_norm;
            ml_candidate = ml;
		}  


		// METRICS - convergence, plots, etc

		double maxdeltalambda = ms.NormInf();
		double maxd			  = lastgoodres;  
			
		// For recording into correction/residuals/violation history, if debugging
		if (this->record_violation_history)
			AtIterationEnd(maxd, maxdeltalambda, iter);

		if (verbose) GetLog() << "  iter=" << iter << "   f=" << mf_p << "  |d|=" << maxd << "  |s|=" << maxdeltalambda  << "\n";


		// Terminate the loop if violation in constraints has been succesfully limited.
		// ***TO DO*** a reliable termination creterion.. 
		/*
		if (maxd < this->tolerance)  
		{
			GetLog() <<"BB premature proj.gradient break at i=" << iter << "\n";
			break;
		}
		*/
		
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

	return lastgoodres;

}








} // END_OF_NAMESPACE____


