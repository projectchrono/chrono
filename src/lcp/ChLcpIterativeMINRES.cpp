///////////////////////////////////////////////////
//
//   ChLcpIterativeMINRES.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
  
   
#include "ChLcpIterativeMINRES.h"


namespace chrono
{

double ChLcpIterativeMINRES::Solve(
					ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
					bool add_Mq_to_f 
					)
{
	std::vector<ChLcpConstraint*>& mconstraints = sysd.GetConstraintsList();
	std::vector<ChLcpVariables*>&  mvariables	= sysd.GetVariablesList();

	double maxviolation = 0.;
	double maxdeltalambda = 0.;
	int i_friction_comp = 0;
	double old_lambda_friction[3];


	// 1)  Update auxiliary data in all constraints before starting,
	//     that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
		mconstraints[ic]->Update_auxiliary();

	// Average all g_i for the triplet of contact constraints n,u,v.
	//
	/*
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
	*/

	// Allocate auxiliary vectors;
	
	int nc = sysd.CountActiveConstraints();

	ChMatrixDynamic<> ml(nc,1);
	ChMatrixDynamic<> mb(nc,1);
	ChMatrixDynamic<> mr(nc,1);
	ChMatrixDynamic<> mp(nc,1);
	ChMatrixDynamic<> mb_i(nc,1);
	ChMatrixDynamic<> Nr(nc,1);
	ChMatrixDynamic<> Np(nc,1);

	// Compute the b_shur vector in the Shur complement equation N*l = b_shur
	// with   
	//   b_shur  = - c + D'*(M\k) = b_i + D'*(M\k)
	//   N_shur  = D'* (M^-1) * D
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
	sysd.BuildBiVector(mb_i);	// b_i   =   -c   = phi/h 
	mb.MatrDec(mb_i);


	// Initialize lambdas
	if (warm_start)
		sysd.FromConstraintsToVector(ml);
	else
		ml.FillElem(0);


	// Compute initial residual
	sysd.ShurComplementProduct(mr, &ml, 0);		// 1)  r = N*l ...
	mr.MatrDec(mb);								// 2)  r = N*l - b_shur
	mr.MatrNeg();								// 3)  r = - N*l + b_shur

	// The MINRES loop:

	mp = mr;

	for (int iter = 0; iter < max_iterations; iter++)
	{
		sysd.ShurComplementProduct(Nr, &mr,0);	// Nr  =  N * r
		double rNr = mr.MatrDot(&mr,&Nr);		// rNr = r' * N * r

		sysd.ShurComplementProduct(Np, &mp,0);	// Np  =  N * p
		double den = Np.NormTwo();				// den =  ((N*p)'(N*p))

		if (den==0) break; 

		double alpha = rNr / den;				// alpha = r'*N*r / ((N*p)'(N*p))

		ml.MatrInc((mp*alpha));					// l = l + alpha * p;
		
		mr.MatrDec((Np*alpha));					// r = r - alpha * N*p;

		sysd.ShurComplementProduct(Nr, &mr,0);	// Nr  =  N * r
		double rNr_ = mr.MatrDot(&mr,&Nr);		// rNr = r' * N * r

		double beta = rNr_ / rNr;				// beta = r'*(N*r)/ rjNrj;
		
		mp.MatrScale(beta);
		mp.MatrInc(mr);							// p = r + beta*p;
	}


	// Store ml temporary vector into ChLcpConstraint 'l_i' multipliers
	sysd.FromConstraintsToVector(ml); 

	// Finally, compute also the primal variables  v = (M^-1)(k - D*l); 

		// v = (M^-1)*k  ...
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
		if (mvariables[iv]->IsActive())
			if (add_Mq_to_f)
				mvariables[iv]->Compute_inc_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb()); // q = q_old + [M]'*fb 
			else
				mvariables[iv]->Compute_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb()); // q = [M]'*fb 

		// ... - (M^-1)*D*l 
	for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
	{	
		if (mconstraints[ic]->IsActive())
			mconstraints[ic]->Increment_q( -mconstraints[ic]->Get_l_i() );
	}
	

	return maxviolation;

}








} // END_OF_NAMESPACE____


