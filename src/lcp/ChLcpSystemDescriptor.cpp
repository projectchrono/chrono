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




int ChLcpSystemDescriptor::CountActiveVariables()
{
	int n_q=0;
	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
	{
		if (vvariables[iv]->IsActive())
		{
			//vvariables[iv]->SetOffset(n_q);	// also store offsets in state and MC matrix
			n_q += vvariables[iv]->Get_ndof();
		}
	}
	return n_q;
}

				
int ChLcpSystemDescriptor::CountActiveConstraints()
{
	int n_c=0;
	for (unsigned int ic = 0; ic< vconstraints.size(); ic++)
	{
		if (vconstraints[ic]->IsActive())
		{
			n_c++;
		}
	}
	return n_c;
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


	// Count active variables, by scanning through all variable blocks,
	// and set offsets

	int n_q=0;
	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
	{
		if (vvariables[iv]->IsActive())
		{
			vvariables[iv]->SetOffset(n_q);	// also store offsets in state and MC matrix
			n_q += vvariables[iv]->Get_ndof();
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


int ChLcpSystemDescriptor::BuildFbVector(
								ChMatrix<>& Fvector	///< matrix which will contain the entire vector of 'f'
						)
{
	int n_q=CountActiveVariables();
	Fvector.Reset(n_q,1);		// fast! Reset() method does not realloc if size doesn't change

	// .. fills F
	int s_q=0;
	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
	{
		if (vvariables[iv]->IsActive())
		{
			Fvector.PasteMatrix(&vvariables[iv]->Get_fb(), s_q, 0);
			s_q += vvariables[iv]->Get_ndof();
		}
	}
	return  s_q; 
}

int ChLcpSystemDescriptor::BuildBiVector(
								ChMatrix<>& Bvector	///< matrix which will contain the entire vector of 'b'
						)
{
	int n_c=CountActiveConstraints();
	Bvector.Resize(n_c, 1);
	
	// Fill the vector
	int s_c=0;
	for (unsigned int ic = 0; ic< vconstraints.size(); ic++)
	{
		if (vconstraints[ic]->IsActive())
		{
			Bvector(s_c) = vconstraints[ic]->Get_b_i();
			++s_c;
		}
	}

	return s_c;
}

void  ChLcpSystemDescriptor::BuildVectors (ChSparseMatrix* f,
								ChSparseMatrix* b,	
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

	f->Reset(n_q, 1);
	b->Reset(n_c, 1);		// fast! Reset() method does not realloc if size doesn't change

	// .. fills f vector of forces
	int s_q=0;
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
	{
		if (mvariables[iv]->IsActive())
		{
			//mvariables[iv]->Get_fb().GetElement(i,0);/////////////////////
			int nn = mvariables[iv]->Get_fb().GetRows();
			for(int ii=0; ii<nn; ii++)
			{
				f->SetElement(s_q+ii,0,mvariables[iv]->Get_fb().GetElement(ii,0));
			}
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
					//mconstraints[ic]->Get_b_i(); ////////////
					b->SetElement(s_c,0,mconstraints[ic]->Get_b_i());
					s_c++;
			  }
	}

}



int ChLcpSystemDescriptor::FromVariablesToVector(	
								ChMatrix<>& mvector,	
								bool resize_vector
								)
{
	// Count active variables and resize vector if necessary
	if (resize_vector)
	{
		int n_q= CountActiveVariables();
		mvector.Resize(n_q, 1);
	}

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

	return  s_q;
}

		

int ChLcpSystemDescriptor::FromVectorToVariables(
								ChMatrix<>& mvector	
								)
{
	#ifdef CH_DEBUG
		int n_q= CountActiveVariables();
		assert(n_q == mvector.GetRows());
		assert(mvector.GetColumns()==1);
	#endif

	// fetch from the vector
	int s_q=0;
	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
	{
		if (vvariables[iv]->IsActive())
		{
			vvariables[iv]->Get_qb().PasteClippedMatrix(&mvector, s_q, 0,  vvariables[iv]->Get_ndof(),1,  0,0);
			s_q += vvariables[iv]->Get_ndof();
		}
	}

	return s_q;
}



int ChLcpSystemDescriptor::FromConstraintsToVector(	
								ChMatrix<>& mvector,
								bool resize_vector
								)
{
	// Count active constraints and resize vector if necessary
	if (resize_vector)
	{
		int n_c=CountActiveConstraints();
		mvector.Resize(n_c, 1);
	}

	// Fill the vector
	int s_c=0;
	for (unsigned int ic = 0; ic< vconstraints.size(); ic++)
	{
		if (vconstraints[ic]->IsActive())
		{
			mvector(s_c) = vconstraints[ic]->Get_l_i();
			++s_c;
		}
	}

	return s_c;
}

		

int ChLcpSystemDescriptor::FromVectorToConstraints(
								ChMatrix<>& mvector	
								)
{
	#ifdef CH_DEBUG
		int n_c=CountActiveConstraints();
		assert(n_c == mvector.GetRows());
		assert(mvector.GetColumns()==1);
	#endif

	// Fill the vector
	int s_c=0;
	for (unsigned int ic = 0; ic< vconstraints.size(); ic++)
	{
		if (vconstraints[ic]->IsActive())
		{
			vconstraints[ic]->Set_l_i(mvector(s_c));
			++s_c;
		}
	}

	return s_c;
}



void ChLcpSystemDescriptor::ShurComplementProduct(	
								ChMatrix<>&	result,	
								ChMatrix<>* lvector,
								std::vector<bool>* enabled  
								)
{
	#ifdef CH_DEBUG
		int n_c=CountActiveConstraints();
		assert(result.GetRows() == n_c);
		assert(result.GetColumns()==1);
		if (enabled) assert(enabled->size() == n_c);
	#endif

	// Performs the sparse product    result = [N]*l = [ [Cq][M^(-1)][Cq'] - [E] ] *l
	// in different phases:

	// 1 - set the qb vector (aka speeds, in each ChLcpVariable sparse data) as zero

	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
		if (vvariables[iv]->IsActive())
			vvariables[iv]->Get_qb().FillElem(0);

	// 2 - performs    qb=[M^(-1)][Cq']*l  by
	//     iterating over all constraints (when implemented in parallel this
	//     could be non-trivial because race conditions might occur -> reduction buffer etc.)
	//     Also, begin to add the cfm term  -[E]*l to the result.

	int s_c=0;
	for (unsigned int ic = 0; ic < vconstraints.size(); ic++)
	{	
		if (vconstraints[ic]->IsActive())
		{
			bool process=true;
			if (enabled)
				if ((*enabled)[s_c]==false)
					process = false;

			if (process) 
			{
				double li;
				if (lvector)
					li = (*lvector)(s_c,0);
				else
					li = vconstraints[ic]->Get_l_i();

				// Compute qb += [M^(-1)][Cq']*l_i
				vconstraints[ic]->Increment_q(li);	// <----!!!  fpu intensive

				// Add constraint force mixing term  result += -[E]*l_i
				result(s_c,0) = - vconstraints[ic]->Get_cfm_i() * li;

			}

			++s_c;
		}
	}

	// 3 - performs    result=[Cq']*qb    by
	//     iterating over all constraints (when implemented in parallel this is trivial)

	s_c=0;
	for (unsigned int ic = 0; ic < vconstraints.size(); ic++)
	{	
		if (vconstraints[ic]->IsActive())
		{
			bool process=true;
			if (enabled)
				if ((*enabled)[s_c]==false)
					process = false;
			
			if (process) 
				result(s_c,0)+= vconstraints[ic]->Compute_Cq_q();	// <----!!!  fpu intensive
			else
				result(s_c,0)= 0; // not enabled constraints, just set to 0 result 
			
			++s_c;
		}
	}

}



} // END_OF_NAMESPACE____


