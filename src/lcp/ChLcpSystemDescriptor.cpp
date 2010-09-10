///////////////////////////////////////////////////
//
//   ChLcpSystemDescriptor.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLcpSystemDescriptor.h"
#include "ChLcpConstraintTwoFrictionT.h"
 
 
namespace chrono 
{


void ChLcpSystemDescriptor::ComputeFeasabilityViolation(
				double& resulting_maxviolation,		///< gets the max constraint violation (either bi- and unilateral.)
				double& resulting_lcpfeasability	///< gets the max feasability as max |l*c| , for unilateral only
				)
{
	resulting_maxviolation = 0;
	resulting_lcpfeasability = 0;

	for (unsigned int ic = 0; ic < vconstraints.size(); ic++)
	{
		// the the residual of the constraint..
		double mres_i = vconstraints[ic]->Compute_c_i();

		double candidate_violation = fabs(vconstraints[ic]->Violation(mres_i));

		if (candidate_violation > resulting_maxviolation)
			resulting_maxviolation = candidate_violation;

		if (vconstraints[ic]->IsUnilateral())
		{
			double candidate_feas = fabs( mres_i * vconstraints[ic]->Get_l_i() ); // =|c*l|
			if (candidate_feas > resulting_lcpfeasability)
				resulting_lcpfeasability = candidate_feas;
		}

	}
}



void  ChLcpSystemDescriptor::BuildMatrices (ChSparseMatrix* Cq,
								ChSparseMatrix* M,	
								bool only_bilaterals, 
								bool skip_contacts_uv)
{
	std::vector<ChLcpConstraint*>& mconstraints = this->GetConstraintsList();
	std::vector<ChLcpVariables*>&  mvariables	= this->GetVariablesList();

	// --
	// Count bilateral and other constraints..

	int n_c=0;
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
	{
		if (mconstraints[ic]->IsActive())
		  if (! ((mconstraints[ic]->GetMode() == CONSTRAINT_FRIC) && only_bilaterals))
			  if ( ! (( dynamic_cast<ChLcpConstraintTwoFrictionT*> (mconstraints[ic])) && skip_contacts_uv))
			  {
				n_c++;
			  }
	}

	// --
	// Count active variables, by scanning through all variable blocks..

	int n_q=0;
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
	{
		if (mvariables[iv]->IsActive())
		{
			mvariables[iv]->SetOffset(n_q);	// also store offsets in state and MC matrix
			n_q += mvariables[iv]->Get_ndof();
		}
	} 

	//if (n_q==0) return;


	// --
	// Reset and resize (if needed) auxiliary vectors

	Cq->Reset(n_c, n_q);
	M->Reset(n_q , n_q);		// fast! Reset() method does not realloc if size doesn't change

	// .. fills M submasses
	int s_q=0;
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
	{
		if (mvariables[iv]->IsActive())
		{
			mvariables[iv]->Build_M(*M, s_q, s_q);				// .. fills  M 
			s_q += mvariables[iv]->Get_ndof();
		}
	}  
	
	// .. fills Cq jacobian
	int s_c=0;
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
	{
		if (mconstraints[ic]->IsActive())
		  if (! ((mconstraints[ic]->GetMode() == CONSTRAINT_FRIC) && only_bilaterals))
			  if ( ! (( dynamic_cast<ChLcpConstraintTwoFrictionT*> (mconstraints[ic])) && skip_contacts_uv))
			  {
					mconstraints[ic]->Build_Cq (*Cq, s_c);// .. fills Cq 
					s_c++;
			  }
	}

}



int ChLcpSystemDescriptor::FromVariablesToVector(	
								ChMatrix<>& mvector						
								)
{
	// Count active variables..
	int n_q=0;
	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
	{
		if (vvariables[iv]->IsActive())
		{
			n_q += vvariables[iv]->Get_ndof();
		}
	}

	mvector.Resize(n_q, 1);

	// Fill the vector
	int s_q=0;
	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
	{
		if (vvariables[iv]->IsActive())
		{
			mvector.PasteMatrix(&vvariables[iv]->Get_qb(), s_q, 0);
			s_q += vvariables[iv]->Get_ndof();
		}
	}

	return n_q;
}

		

int ChLcpSystemDescriptor::FromVectorToVariables(
								ChMatrix<>& mvector	
								)
{
	// Count active variables..
	int n_q=0;
	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
	{
		if (vvariables[iv]->IsActive())
		{
			n_q += vvariables[iv]->Get_ndof();
		}
	}
	assert(n_q == mvector.GetRows());
	assert(mvector.GetColumns()==1);

	// fetch from the vector
	int s_q=0;
	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
	{
		if (vvariables[iv]->IsActive())
		{
			//vvariables[iv]->Get_qb().PasteClippedMatrix(&mvector, s_q, 0,  vvariables[iv]->Get_ndof(),1,  0,0);
			s_q += vvariables[iv]->Get_ndof();
		}
	}

	return n_q;
}









} // END_OF_NAMESPACE____


