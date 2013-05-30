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
#include "ChLcpConstraintTwoRollingN.h"
#include "ChLcpConstraintTwoRollingT.h"



 
namespace chrono 
{


ChLcpSystemDescriptor::ChLcpSystemDescriptor()
{
	vconstraints.clear();
	vvariables.clear();

	this->num_threads = 0;
	solver_threads = 0;
	SetNumThreads(this->num_threads);
/*
	locktable = new ChMutexSpinlock[100];
*/
};


ChLcpSystemDescriptor::~ChLcpSystemDescriptor()
{
	vconstraints.clear();
	vvariables.clear();

	if (solver_threads) 
		delete (solver_threads); 
	solver_threads =0;

/*
	if (locktable)
		delete[] locktable;
	locktable=0;
*/
};







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
			vvariables[iv]->SetOffset(n_q);	// also store offsets in state and MC matrix
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


void ChLcpSystemDescriptor::ConvertToMatrixForm (
								  ChSparseMatrix* Cq, 
								  ChSparseMatrix* M, 
								  ChSparseMatrix* E,  
								  ChMatrix<>* Fvector,
								  ChMatrix<>* Bvector,
								  ChMatrix<>* Frict,  
								bool only_bilaterals, 
								bool skip_contacts_uv)
{
	std::vector<ChLcpConstraint*>& mconstraints = this->GetConstraintsList();
	std::vector<ChLcpVariables*>&  mvariables	= this->GetVariablesList();

	// Count bilateral and other constraints.. (if wanted, bilaterals only)

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

	int n_q = this->CountActiveVariables();

	
	// Reset and resize (if needed) auxiliary vectors

	if (Cq) 
		Cq->Reset(n_c, n_q);
	if (M)
		M->Reset (n_q, n_q);
	if (E)
		E->Reset (n_c, n_c);
	if (Fvector)
		Fvector->Reset (n_q, 1);
	if (Bvector)
		Bvector->Reset (n_c, 1);
	if (Frict)
		Frict->Reset (n_c, 1);

	// Fills M submasses and 'f' vector,
	// by looping on variables
	int s_q=0;
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
	{
		if (mvariables[iv]->IsActive())
		{
			if (M)
				mvariables[iv]->Build_M(*M, s_q, s_q);				// .. fills  M 
			if (Fvector)
				Fvector->PasteMatrix(&vvariables[iv]->Get_fb(), s_q, 0);// .. fills  'f' 
			s_q += mvariables[iv]->Get_ndof();
		}
	}  
	
	// If some stiffness / hessian matrix has been added to M ,
	// also add it to the sparse M
	int s_k=0;
	for (unsigned int ik = 0; ik< this->vstiffness.size(); ik++)
	{
		this->vstiffness[ik]->Build_K(*M, true); 
	} 

	// Fills Cq jacobian, E 'compliance' matrix , the 'b' vector and friction coeff.vector, 
	// by looping on constraints
	int s_c=0;
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
	{
		if (mconstraints[ic]->IsActive())
		  if (! ((mconstraints[ic]->GetMode() == CONSTRAINT_FRIC) && only_bilaterals))
			  if ( ! (( dynamic_cast<ChLcpConstraintTwoFrictionT*> (mconstraints[ic])) && skip_contacts_uv))
			  {
					if (Cq)
						mconstraints[ic]->Build_Cq (*Cq, s_c);// .. fills Cq
					if (E)
						E->SetElement(s_c,s_c, - mconstraints[ic]->Get_cfm_i() ); // .. fills E ( = - cfm )
					if (Bvector)
						(*Bvector)(s_c) = mconstraints[ic]->Get_b_i(); // .. fills 'b'   
					if (Frict) // .. fills vector of friction coefficients
					{
						(*Frict)(s_c) = -2; // mark with -2 flag for bilaterals (default)
						if (ChLcpConstraintTwoContactN* mcon = dynamic_cast<ChLcpConstraintTwoContactN*> (mconstraints[ic])) 
							(*Frict)(s_c) = mcon->GetFrictionCoefficient(); // friction coeff only in row of normal component 
						if (ChLcpConstraintTwoFrictionT* mcon = dynamic_cast<ChLcpConstraintTwoFrictionT*> (mconstraints[ic])) 
							(*Frict)(s_c) = -1; // mark with -1 flag for rows of tangential components
					}
					s_c++;
			  }
	}

}

void  ChLcpSystemDescriptor::BuildMatrices (ChSparseMatrix* Cq,
								ChSparseMatrix* M,	
								bool only_bilaterals, 
								bool skip_contacts_uv)
{
	this->ConvertToMatrixForm(Cq,M,0,0,0,0,only_bilaterals,skip_contacts_uv);
}
void  ChLcpSystemDescriptor::BuildVectors (ChSparseMatrix* f,
								ChSparseMatrix* b,	
								bool only_bilaterals, 
								bool skip_contacts_uv)
{
	this->ConvertToMatrixForm(0,0,0,f,b,0,only_bilaterals,skip_contacts_uv);
}

int ChLcpSystemDescriptor::BuildFbVector(
								ChMatrix<>& Fvector	///< matrix which will contain the entire vector of 'f'
						)
{
	int n_q=CountActiveVariables();
	Fvector.Reset(n_q,1);		// fast! Reset() method does not realloc if size doesn't change

	// Fills the 'f' vector
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
	
	// Fill the 'b' vector
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


int ChLcpSystemDescriptor::BuildDiVector(
								ChMatrix<>& Dvector	
						)
{
	int n_q=CountActiveVariables();
	int n_c=CountActiveConstraints();

	Dvector.Reset(n_q+n_c,1);		// fast! Reset() method does not realloc if size doesn't change

	// Fills the 'f' vector part
	int s_u=0;
	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
	{
		if (vvariables[iv]->IsActive())
		{
			Dvector.PasteMatrix(&vvariables[iv]->Get_fb(), s_u, 0);
			s_u += vvariables[iv]->Get_ndof();
		}
	}
	// Fill the '-b' vector (with flipped sign!)
	for (unsigned int ic = 0; ic< vconstraints.size(); ic++)
	{
		if (vconstraints[ic]->IsActive())
		{
			Dvector(s_u) = -vconstraints[ic]->Get_b_i();
			++s_u;
		}
	}

	return  s_u; 
}

int  ChLcpSystemDescriptor::BuildDiagonalVector(
								ChMatrix<>& Diagonal_vect  	///< matrix which will contain the entire vector of terms on M and E diagonal
							)
{
	int n_q=CountActiveVariables();
	int n_c=CountActiveConstraints();

	Diagonal_vect.Reset(n_q+n_c,1);		// fast! Reset() method does not realloc if size doesn't change

	// Fill the diagonal values given by stiffness blocks, if any
	for (unsigned int is = 0; is< vstiffness.size(); is++)
	{
		vstiffness[is]->DiagonalAdd(Diagonal_vect);
	}
	// Get the 'M' diagonal terms
	int s_u=0;
	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
	{
		if (vvariables[iv]->IsActive())
		{
			vvariables[iv]->DiagonalAdd(Diagonal_vect);
			s_u += vvariables[iv]->Get_ndof();
		}
	}
	// Get the 'E' diagonal terms (note the sign: E_i = -cfm_i )
	for (unsigned int ic = 0; ic< vconstraints.size(); ic++)
	{
		if (vconstraints[ic]->IsActive())
		{
			Diagonal_vect(s_u) = -vconstraints[ic]->Get_cfm_i();
			++s_u;
		}
	}
	return s_u;
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


int ChLcpSystemDescriptor::FromUnknownsToVector(	
								ChMatrix<>& mvector,	
								bool resize_vector
								)
{
	// Count active variables & constraints and resize vector if necessary
	if (resize_vector)
	{
		int n_q= CountActiveVariables();
		int n_c= CountActiveConstraints();
		mvector.Resize(n_q+n_c, 1);
	}

	// Fill the first part of vector, x.q ,with variables q
	int s_u=0;
	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
	{
		if (vvariables[iv]->IsActive())
		{
			mvector.PasteMatrix(&vvariables[iv]->Get_qb(), s_u, 0);
			s_u += vvariables[iv]->Get_ndof();
		}
	}
	// Fill the second part of vector, x.l, with constraint multipliers -l (with flipped sign!)
	for (unsigned int ic = 0; ic< vconstraints.size(); ic++)
	{
		if (vconstraints[ic]->IsActive())
		{
			mvector(s_u) = -vconstraints[ic]->Get_l_i();
			++s_u;
		}
	}

	return  s_u;
}

		

int ChLcpSystemDescriptor::FromVectorToUnknowns(
								ChMatrix<>& mvector	
								)
{
	#ifdef CH_DEBUG
		int n_q= CountActiveVariables();
		assert(n_q == mvector.GetRows());
		assert(mvector.GetColumns()==1);
	#endif

	// fetch from the first part of vector (x.q = q)
	int s_u=0;
	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
	{
		if (vvariables[iv]->IsActive())
		{
			vvariables[iv]->Get_qb().PasteClippedMatrix(&mvector, s_u, 0,  vvariables[iv]->Get_ndof(),1,  0,0);
			s_u += vvariables[iv]->Get_ndof();
		}
	}
	// fetch from the second part of vector (x.l = -l), with flipped sign!
	for (unsigned int ic = 0; ic< vconstraints.size(); ic++)
	{
		if (vconstraints[ic]->IsActive())
		{
			vconstraints[ic]->Set_l_i( -mvector(s_u));
			++s_u;
		}
	}

	return s_u;
}




struct thread_pdata 
{
	ChLcpSystemDescriptor* descriptor;		// reference to sys.descriptor 
	ChMutexSpinlock* mutex;	// this will be used to avoid race condition when writing to shared memory.
	
	ChMatrix<>* result;
	ChMatrix<>* lvector;
	std::vector<bool>* enabled;

	enum solver_stage
	{
		STAGE1_PRODUCT = 0,
		STAGE2_PRODUCT,
	};
	solver_stage stage;

				// the range of scanned multipliers (for loops on multipliers)
	unsigned int constr_from;	
	unsigned int constr_to;
	unsigned int cont_stride; // in result vector where inactive constr. are squeezed away
		
				// the range of scanned 'variables' objects (for loops on variables, aka rigid bodies)
	unsigned int var_from;
	unsigned int var_to;

	std::vector<ChLcpConstraint*>* mconstraints;
	std::vector<ChLcpVariables*>*  mvariables;
	bool madd_Mq_to_f;
};

// Don't create local store memory, just return 0

void*	SystemdMemoryFunc()
{
	return 0;
}


// The following is the function which will be executed by 
// each thread, when threads are launched at each Solve()  


void SystemdThreadFunc(void* userPtr,void* lsMemory)
{
//	GetLog() << " SystemdThreadFunc \n";
	thread_pdata* tdata = (thread_pdata*)userPtr;

	std::vector<ChLcpConstraint*>* vconstraints = tdata->mconstraints;
	std::vector<ChLcpVariables*>*  vvariables = tdata->mvariables;
	bool madd_Mq_to_f = tdata->madd_Mq_to_f;
	ChMatrix<>*	result = tdata->result;
	ChMatrix<>* lvector = tdata->lvector;
	std::vector<bool>* enabled = tdata->enabled;

	switch (tdata->stage)
	{
	case thread_pdata::STAGE1_PRODUCT:
		{
		//	GetLog() << " STAGE1_PRODUCT  from " << tdata->constr_from << " to " <<   tdata->constr_to << "\n";

			int s_c= tdata->cont_stride;
			for (unsigned int ic = tdata->constr_from; ic< tdata->constr_to; ic++)
			{	
				if ((*vconstraints)[ic]->IsActive())
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
							li = (*vconstraints)[ic]->Get_l_i();

						// Compute qb += [M^(-1)][Cq']*l_i

					//int it = ..some hashed value here..; //***TO DO***
					//((tdata->mutex)+it)->Lock();
					
						(*vconstraints)[ic]->Increment_q(li);	// <----!!!  fpu intensive
					
					//((tdata->mutex)+it)->Unlock();  //***TO DO***

						// Add constraint force mixing term  result = cfm * l_i = -[E]*l_i
						(*result)(s_c,0) = (*vconstraints)[ic]->Get_cfm_i() * li;

					}
					++s_c;
				}
			}
			break;  // end stage
		}

	case thread_pdata::STAGE2_PRODUCT:
		{
		//	GetLog() << " STAGE2_PRODUCT  from " << tdata->constr_from << " to " <<   tdata->constr_to << "\n";

			int s_c= tdata->cont_stride;
			for (unsigned int ic = tdata->constr_from; ic< tdata->constr_to; ic++)
			{	
				if ((*vconstraints)[ic]->IsActive())
				{
					bool process=true;
					if (enabled)
						if ((*enabled)[s_c]==false)
							process = false;
					
					if (process) 
						(*result)(s_c,0)+= (*vconstraints)[ic]->Compute_Cq_q();	// <----!!!  fpu intensive
					else
						(*result)(s_c,0)= 0; // not enabled constraints, just set to 0 result 
					
					++s_c;
				}
			}
			break;  // end stage
		}

	default:
		{
			break;
		}

	} // end stage  switching

}



#define CH_SERIAL_SHUR
//#define CH_PARALLEL_SHUR
//#define CH_PARALLEL_TEST


void ChLcpSystemDescriptor::ShurComplementProduct(	
								ChMatrix<>&	result,	
								ChMatrix<>* lvector,
								std::vector<bool>* enabled  
								)
{
	assert(this->vstiffness.size() == 0); // currently, the case with ChLcpKstiffness items is not supported (only diagonal M is supported, no K)

	#ifdef CH_DEBUG
		int n_c=CountActiveConstraints();
		assert(result.GetRows() == n_c);
		assert(result.GetColumns()==1);
		if (enabled) assert(enabled->size() == n_c);
	#endif


	#ifdef CH_SERIAL_SHUR

	// Performs the sparse product    result = [N]*l = [ [Cq][M^(-1)][Cq'] - [E] ] *l
	// in different phases:

	// 1 - set the qb vector (aka speeds, in each ChLcpVariable sparse data) as zero

	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
		if (vvariables[iv]->IsActive())
			vvariables[iv]->Get_qb().FillElem(0);

	// 2 - performs    qb=[M^(-1)][Cq']*l  by
	//     iterating over all constraints (when implemented in parallel this
	//     could be non-trivial because race conditions might occur -> reduction buffer etc.)
	//     Also, begin to add the cfm term ( -[E]*l ) to the result.

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

				// Add constraint force mixing term  result = cfm * l_i = -[E]*l_i
				result(s_c,0) =  vconstraints[ic]->Get_cfm_i() * li;

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

	#endif

	#ifdef CH_PARALLEL_TEST
	ChMatrixDynamic<> TEST_lvector(lvector->GetRows(),1);
	TEST_lvector.CopyFromMatrix(*lvector);
	ChMatrixDynamic<> TEST_result(result.GetRows(),1);
	TEST_result.CopyFromMatrix(result);
	ChMatrixDynamic<> TEST_diff(result.GetRows(),1);
	TEST_diff.CopyFromMatrix(result);
	TEST_diff.Reset();
	#endif

	#ifdef CH_PARALLEL_SHUR

	// set the qb vector (aka speeds, in each ChLcpVariable sparse data) as zero

	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
		if (vvariables[iv]->IsActive())
			vvariables[iv]->Get_qb().FillElem(0);

	//ChMutexSpinlock spinlock;

	// Preparation: 
	//        subdivide the workload to the threads and prepare their 'thread_data':

	int numthreads = this->num_threads;
	std::vector<thread_pdata> mdataN(numthreads);

//static ChMutexSpinlock* alocktable = new ChMutexSpinlock[1];

	int var_slice = 0;
	int constr_slice = 0;
	for (int nth = 0; nth < numthreads; nth++)
	{
		unsigned int var_from = var_slice;
		unsigned int var_to = var_from + (vvariables.size()-var_from) / (numthreads-nth);
		unsigned int constr_from = constr_slice;
		unsigned int constr_to = constr_from + (vconstraints.size()-constr_from) / (numthreads-nth);
		if(constr_to < vconstraints.size()) // do not slice the three contact multipliers (or six in case of rolling)
		{
			if (dynamic_cast<ChLcpConstraintTwoFrictionT*>(vconstraints[constr_to]))
				constr_to++;
			if (dynamic_cast<ChLcpConstraintTwoFrictionT*>(vconstraints[constr_to]))
				constr_to++;
			if (constr_to < vconstraints.size())
			{
				if (dynamic_cast<ChLcpConstraintTwoRollingN*>(vconstraints[constr_to]))
					constr_to++;
				if (dynamic_cast<ChLcpConstraintTwoRollingT*>(vconstraints[constr_to]))
					constr_to++;
				if (dynamic_cast<ChLcpConstraintTwoRollingT*>(vconstraints[constr_to]))
					constr_to++;
			}
		}
		
		mdataN[nth].descriptor = this;
		mdataN[nth].mutex = this->locktable;
		mdataN[nth].constr_from = constr_from;
		mdataN[nth].constr_to   = constr_to;
		mdataN[nth].var_from = var_from;
		mdataN[nth].var_to = var_to;
		mdataN[nth].mconstraints = &vconstraints;
		mdataN[nth].mvariables = &vvariables;
		mdataN[nth].result = &result;
		mdataN[nth].lvector = lvector;
		mdataN[nth].enabled = enabled;

		var_slice = var_to;
		constr_slice = constr_to;
	}

	// compute strides for vectors, because some constr. might be inactive
	int st_c=0;
	int in_th=0;
	for (unsigned int ic = 0; ic< vconstraints.size(); ic++)
	{
		if (ic == mdataN[in_th].constr_from)
		{
			mdataN[in_th].cont_stride=st_c;
			++in_th;
		}
		if (in_th >= numthreads) 
			break;

		if (vconstraints[ic]->IsActive())
			++st_c;
	}

	// LAUNCH THE PARALLEL COMPUTATION ON THREADS !!!!

	// --1--  stage:  
	for (int nth = 0; nth < numthreads; nth++)
	{
		mdataN[nth].stage = thread_pdata::STAGE1_PRODUCT;
		solver_threads->sendRequest(1, &mdataN[nth], nth);
	}
	//... must wait that the all the threads finished their stage!!!
	solver_threads->flush();

	// --2--  stage:  
	for (int nth = 0; nth < numthreads; nth++)
	{
		mdataN[nth].stage = thread_pdata::STAGE2_PRODUCT;
		solver_threads->sendRequest(1, &mdataN[nth], nth);
	}
	//... must wait that the all the threads finished their stage!!!
	solver_threads->flush();


	#endif

	#ifdef CH_PARALLEL_TEST
	 TEST_diff.MatrSub(result, TEST_result);
	 double mdiff = TEST_diff.NormInf();
	 if (mdiff>10e-14) 
		GetLog() << "ERROR! product diff = " << TEST_diff.NormInf() << "\n";
	#endif


}




void ChLcpSystemDescriptor::SystemProduct(	
								ChMatrix<>&	result,			///< matrix which contains the result of matrix by x 
								ChMatrix<>* x		        ///< optional matrix with the vector to be multiplied (if null, use current l_i and q)
								// std::vector<bool>* enabled=0 ///< optional: vector of enable flags, one per scalar constraint. true=enable, false=disable (skip)
								)
{
	int n_q = this->CountActiveVariables();
	int n_c = this->CountActiveConstraints();

	ChMatrix<>* x_ql = 0;

	ChMatrix<>* vect;

	if (x)
		vect = x;
	else
	{
		x_ql = new ChMatrixDynamic<double>(n_q+n_c,1);
		vect = x_ql;
		this->FromUnknownsToVector(*vect);
	}

	result.Reset(n_q+n_c,1); // fast! Reset() method does not realloc if size doesn't change

	// 1) First row: result.q part =  [M + K]*x.q + [Cq']*x.l

	// 1.1)  do  M*x.q
	for (unsigned int iv = 0; iv< vvariables.size(); iv++)
		if (vvariables[iv]->IsActive())
		{
			vvariables[iv]->MultiplyAndAdd(result,*x);
		}

	// 1.2)  add also K*x.q
	for (unsigned int ik = 0; ik< vstiffness.size(); ik++)
	{
		vstiffness[ik]->MultiplyAndAdd(result,*x);
	}

	// 1.3)  add also [Cq]'*x.l
	int s_c= n_q;
	for (unsigned int ic = 0; ic < vconstraints.size(); ic++)
	{	
		if (vconstraints[ic]->IsActive())
		{
			vconstraints[ic]->MultiplyTandAdd(result,  (*x)(s_c));
			++s_c;
		}
	}

	// 2) Second row: result.l part =  [C_q]*x.q + [E]*x.l
	s_c= n_q;
	for (unsigned int ic = 0; ic < vconstraints.size(); ic++)
	{	
		if (vconstraints[ic]->IsActive())
		{
			vconstraints[ic]->MultiplyAndAdd(result(s_c), (*x));  // result.l_i += [C_q_i]*x.q
			result(s_c) -= vconstraints[ic]->Get_cfm_i()* (*x)(s_c); // result.l_i += [E]*x.l_i  NOTE:  cfm = -E
			++s_c;
		}
	}		
	


	// if a temp vector has been created because x was not provided, then delete it
	if (x_ql)
		delete x_ql;
}




void ChLcpSystemDescriptor::ConstraintsProject(	
								ChMatrix<>&	multipliers		///< matrix which contains the entire vector of 'l_i' multipliers to be projected
								)
{
	this->FromVectorToConstraints(multipliers);		
		for (unsigned int ic = 0; ic < vconstraints.size(); ic++)
					if (vconstraints[ic]->IsActive())
							vconstraints[ic]->Project();
	this->FromConstraintsToVector(multipliers,false);		
}


void ChLcpSystemDescriptor::UnknownsProject(	
								ChMatrix<>&	mx		///< matrix which contains the entire vector of unknowns x={q,-l} (only the l part is projected)
								)
{
	int s_u= this->CountActiveVariables();
	int s_u_bk = s_u;

	// vector -> constraints
	// Fetch from the second part of vector (x.l = -l), with flipped sign!
	for (unsigned int ic = 0; ic< vconstraints.size(); ic++)
	{
		if (vconstraints[ic]->IsActive())
		{
			vconstraints[ic]->Set_l_i( -mx(s_u));
			++s_u;
		}
	}
		
	// constraint projection!
	for (unsigned int ic = 0; ic < vconstraints.size(); ic++)
			if (vconstraints[ic]->IsActive())
					vconstraints[ic]->Project();

	// constraints -> vector
	// Fill the second part of vector, x.l, with constraint multipliers -l (with flipped sign!)
	s_u = s_u_bk;
	for (unsigned int ic = 0; ic< vconstraints.size(); ic++)
	{
		if (vconstraints[ic]->IsActive())
		{
			mx(s_u) = -vconstraints[ic]->Get_l_i();
			++s_u;
		}
	}	
}




void ChLcpSystemDescriptor::SetNumThreads(int nthreads) 
{

	if (nthreads == this->num_threads)
		return;

	this->num_threads = nthreads;

	// Delete the threads container
	if (this->solver_threads) 
		delete (solver_threads); 
	solver_threads =0;

	if (this->num_threads==0)
		return;

	// Create the threads container
	ChThreadConstructionInfo create_args ( (char*)"solver",
						SystemdThreadFunc, 
						SystemdMemoryFunc, 
						this->num_threads);

	this->solver_threads = new ChThreads(create_args); 

	/* not needed?
	if (locktable)
		delete[] locktable;

	locktable = new ChMutexSpinlock[100];
	*/
}


} // END_OF_NAMESPACE____


