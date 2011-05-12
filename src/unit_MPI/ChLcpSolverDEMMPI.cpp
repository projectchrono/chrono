///////////////////////////////////////////////////
//
//   ChLcpSolverDEMMPI.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
  
   
#include "ChLcpSolverDEMMPI.h"
#include "ChLcpSystemDescriptorMPI.h"
#include "ChMpi.h"

namespace chrono
{

double ChLcpSolverDEMMPI::Solve(
					ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
					bool add_Mq_to_f 
					)
{

	// Should work only with ChLcpSystemDescriptorMPI..
	assert (dynamic_cast<ChLcpSystemDescriptorMPI*>(&sysd));

	std::vector<ChLcpVariables*>&  mvariables	= sysd.GetVariablesList();

	double maxviolation = 0.;

	// 2)  Compute, for all items with variables, the initial guess for
	//     still unconstrained system:

	ChMatrixDynamic<> previous_q;
	if (add_Mq_to_f)
		sysd.FromVariablesToVector(previous_q);

	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
		if (mvariables[iv]->IsActive())
			if (add_Mq_to_f)
				mvariables[iv]->Compute_inc_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb()); // q = q_old + [M]'*fb 
			else
				mvariables[iv]->Compute_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb()); // q = [M]'*fb 


	// here send, then receive all infos from other domains,
	// adding neighbouring effect to the q vector (like it is an 'apply forces').
	ChLcpSystemDescriptorMPI* domain_desc = dynamic_cast<ChLcpSystemDescriptorMPI*>(&sysd);
	domain_desc->PerformCommunication(); 

	return maxviolation;

}








} // END_OF_NAMESPACE____


