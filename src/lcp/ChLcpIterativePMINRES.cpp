///////////////////////////////////////////////////
//
//   ChLcpIterativePMINRES.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
  
   
#include "ChLcpIterativePMINRES.h"
#include "ChLcpConstraintTwoFrictionT.h"

namespace chrono
{

double ChLcpIterativePMINRES::Solve(
					ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
					bool add_Mq_to_f 
					)
{
	std::vector<ChLcpConstraint*>& mconstraints = sysd.GetConstraintsList();
	std::vector<ChLcpVariables*>&  mvariables	= sysd.GetVariablesList();

	double maxviolation = 0.;

	
	bool verbose = true;


	// Update auxiliary data in all constraints before starting,
	// that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
		mconstraints[ic]->Update_auxiliary();


	// Allocate auxiliary vectors;
	
	int nc = sysd.CountActiveConstraints();
	if (verbose) GetLog() <<"\n-----Projected MINRES, solving nc=" << nc << "unknowns \n";

	ChMatrixDynamic<> ml(nc,1);
	ChMatrixDynamic<> mb(nc,1);
	ChMatrixDynamic<> mp(nc,1);
	ChMatrixDynamic<> mr(nc,1);
	ChMatrixDynamic<> mr_old(nc,1);
	ChMatrixDynamic<> mNp(nc,1);
	ChMatrixDynamic<> mNr(nc,1);
	ChMatrixDynamic<> mNr_old(nc,1);
	ChMatrixDynamic<> mtmp(nc,1);


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

	// ...and now do  b_shur = - D' * q  ..
	int s_i = 0;
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
		if (mconstraints[ic]->IsActive())
		{
			mb(s_i, 0) = - mconstraints[ic]->Compute_Cq_q();
			++s_i;
		}

	// ..and finally do   b_shur = b_shur - c
	sysd.BuildBiVector(mtmp);	// b_i   =   -c   = phi/h 
	mb.MatrDec(mtmp);  


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
	// ...

	// r = b - N*l;
	sysd.ShurComplementProduct(mr, &ml);		// 1)  r = N*l ...        #### MATR.MULTIPLICATION!!!###
	mr.MatrNeg();								// 2)  r =-N*l
	mr.MatrInc(mb);								// 3)  r =-N*l+b

	// p = r;
	mp = mr;
	
	// Nr = N*r
	sysd.ShurComplementProduct(mNr, &mr);		// Nr = N*r    #### MATR.MULTIPLICATION!!!###



	//
	// THE LOOP
	//

	std::vector<double> f_hist;

	for (int iter = 0; iter < max_iterations; iter++)
	{
		// Np = N*p;
		sysd.ShurComplementProduct(mNp, &mp);		// Np = N*p    #### MATR.MULTIPLICATION!!!###

		// alpha = r'*(Nr)/((Np)'*(Np));  // alpha =  u'*p / p'*N*p 
		double rNr =  mr.MatrDot(&mr,&mNr);			// 1)  rNr = r'*N*r
		double NpNp = mNp.MatrDot(&mNp,&mNp);		// 2)  NpNp = ((Np)'*(Np))
		 if (fabs(NpNp)<10e-16) 
		 {
			if (verbose) GetLog() << "Rayleygh quotient alpha breakdown: " << rNr << " / " << NpNp << "  iter=" << iter << "\n";
			NpNp=10e-16;
		 }
		double alpha = rNr/NpNp;					// 3)  alpha = r'*(Nr)/((Np)'*(Np))

		// l = l + alpha * p;
		mtmp.CopyFromMatrix(mp);
		mtmp.MatrScale(alpha);
		ml.MatrInc(mtmp);

		double maxdeltalambda = mtmp.NormInf();

		if (maxdeltalambda < this->GetTolerance() ) 
		{
			if (verbose) GetLog() << "Converged! iter=" << iter <<  "\n";
		 	//break;
		}

		// l = Proj(l)
		sysd.ConstraintsProject(ml);				// l = P(l) 

		// r_old = r;
		mr_old.CopyFromMatrix(mr);

		// r = b - N*l;
		sysd.ShurComplementProduct(mr, &ml);		// 1)  r = N*l ...        #### MATR.MULTIPLICATION!!!###
		mr.MatrNeg();								// 2)  r =-N*l
		mr.MatrInc(mb);								// 3)  r =-N*l+b

		
		// r = (project_orthogonal(l+diff*r, fric) - l)/diff; 
		mr.MatrScale(this->grad_diffstep);
		mr.MatrInc(ml);
		sysd.ConstraintsProject(mr);				// r = P(l+diff*r) ...
		mr.MatrDec(ml);
		mr.MatrScale(1.0/this->grad_diffstep);		// r = (P(l+diff*r)-l)/diff 

		// p = (project_orthogonal(l+diff*p, fric) - l)/diff;
		mp.MatrScale(this->grad_diffstep);
		mp.MatrInc(ml);
		sysd.ConstraintsProject(mr);				// p = P(l+diff*p) ...
		mp.MatrDec(ml);
		mp.MatrScale(1.0/this->grad_diffstep);		// p = (P(l+diff*p)-l)/diff 

		// Nr_old = Nr;
		mNr_old.CopyFromMatrix(mNr);

		// Nr = N*r; 
		sysd.ShurComplementProduct(mNr, &mr);		// Nr = N*r    #### MATR.MULTIPLICATION!!!###

		// beta = r'*(Nr-Nr_old)/(r_old'*(Nr_old));
		mtmp.MatrSub(mNr,mNr_old);
		double numerator = mr.MatrDot(&mr,&mtmp);
		double denominator = mr_old.MatrDot(&mr_old, &mNr_old);
		if (fabs(denominator)<10e-16)
		 {
			if (verbose) GetLog() << "Rayleygh quotient beta breakdown: " << numerator << " / " << denominator <<  "  iter=" << iter << "\n";
			denominator=10e-16;
		 }
		double beta = numerator / denominator;
		
		beta = ChMax(0.0, beta);

		

		// p = r + beta * p;
		mtmp.CopyFromMatrix(mp);
		mtmp.MatrScale(beta);
		mp.CopyFromMatrix(mr);
		mp.MatrInc(mtmp);


		// METRICS - convergence, plots, etc
		//double maxd			  = mu.NormInf();  // ***TO DO***  should be max violation, but just for test...
		double maxd=0; // ***TO DO***

		// For recording into correction/residuals/violation history, if debugging
		if (this->record_violation_history)
			AtIterationEnd(maxd, maxdeltalambda, iter);
			
	}
	

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


