///////////////////////////////////////////////////
//
//   ChLcpSolverDEM.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
  
   
#include "ChLcpSolverDEM.h"


namespace chrono
{

double ChLcpSolverDEM::Solve(
					ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
					bool add_Mq_to_f 
					)
{
	std::vector<ChLcpVariables*>&  mvariables	= sysd.GetVariablesList();

	double maxviolation = 0.;
	double maxdeltalambda = 0.;
	int i_friction_comp = 0;
	double old_lambda_friction[3];


	// 2)  Compute, for all items with variables, the initial guess for
	//     still unconstrained system:

	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
		if (mvariables[iv]->IsActive())
			if (add_Mq_to_f)
				mvariables[iv]->Compute_inc_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb()); // q = q_old + [M]'*fb 
			else
				mvariables[iv]->Compute_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb()); // q = [M]'*fb 

	return maxviolation;

}








} // END_OF_NAMESPACE____


