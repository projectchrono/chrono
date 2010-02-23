///////////////////////////////////////////////////
//
//   ChLcpSolver.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

  
#include "ChLcpSolver.h"
 
 
namespace chrono 
{



void ChLcpSolver::ComputeFeasabilityViolation(
				std::vector<ChLcpConstraint*>& mconstraints,///< array of pointers to constraints
				double& resulting_maxviolation,		///< gets the max constraint violation (either bi- and unilateral.)
				double& resulting_lcpfeasability	///< gets the max feasability as max |l*c| , for unilateral only
				)
{
	resulting_maxviolation = 0;
	resulting_lcpfeasability = 0;

	for (unsigned int ic = 0; ic < mconstraints.size(); ic++)
	{
		// the the residual of the constraint..
		double mres_i = mconstraints[ic]->Compute_c_i();

		double candidate_violation = fabs(mconstraints[ic]->Violation(mres_i));

		if (candidate_violation > resulting_maxviolation)
			resulting_maxviolation = candidate_violation;

		if (mconstraints[ic]->IsUnilateral())
		{
			double candidate_feas = fabs( mres_i * mconstraints[ic]->Get_l_i() ); // =|c*l|
			if (candidate_feas > resulting_lcpfeasability)
				resulting_lcpfeasability = candidate_feas;
		}

	}
}




int ChLcpSolver::FromVariablesToVector(
								std::vector<ChLcpVariables*>& mvariables,	
								ChMatrix<>& mvector						
								)
{
	// Count active variables..
	int n_q=0;
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
	{
		if (mvariables[iv]->IsActive())
		{
			n_q += mvariables[iv]->Get_ndof();
		}
	}

	mvector.Resize(n_q, 1);

	// Fill the vector
	int s_q=0;
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
	{
		if (mvariables[iv]->IsActive())
		{
			mvector.PasteMatrix(&mvariables[iv]->Get_qb(), s_q, 0);
			s_q += mvariables[iv]->Get_ndof();
		}
	}

	return n_q;
}

		

int ChLcpSolver::FromVectorToVariables(
								ChMatrix<>& mvector	,					
								std::vector<ChLcpVariables*>& mvariables	
								)
{
	// Count active variables..
	int n_q=0;
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
	{
		if (mvariables[iv]->IsActive())
		{
			n_q += mvariables[iv]->Get_ndof();
		}
	}
	assert(n_q == mvector.GetRows());
	assert(mvector.GetColumns()==1);

	// fetch from the vector
	int s_q=0;
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
	{
		if (mvariables[iv]->IsActive())
		{
			mvariables[iv]->Get_qb().PasteClippedMatrix(&mvector, s_q, 0,  mvariables[iv]->Get_ndof(),1,  0,0);
			mvector.PasteMatrix(&mvariables[iv]->Get_qb(), s_q, 0);
			s_q += mvariables[iv]->Get_ndof();
		}
	}

	return n_q;
}



} // END_OF_NAMESPACE____


