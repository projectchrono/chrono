///////////////////////////////////////////////////
//
//   ChLcpSimplexSolver.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLcpSimplexSolver.h"
#include "core/ChSpmatrix.h"
 
 
namespace chrono 
{

ChLcpSimplexSolver::ChLcpSimplexSolver()
{
	MC = new ChSparseMatrix(30,30); // at least as big as 30x30
	X  = new ChMatrixDynamic<>(30,1); // at least as big as 30x1
	B  = new ChMatrixDynamic<>(30,1); // at least as big as 30x1
	unilaterals = 0;
	truncation_step=0;
}
				
ChLcpSimplexSolver::~ChLcpSimplexSolver()
{
	if (MC) delete MC; MC=0;
	if (X) delete X; X=0;
	if (B) delete B; B=0;
	if (unilaterals) delete[]unilaterals;
}
 


double ChLcpSimplexSolver::Solve(
					ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
					bool add_Mq_to_f  
					)
{
	std::vector<ChLcpConstraint*>& mconstraints = sysd.GetConstraintsList();
	std::vector<ChLcpVariables*>&  mvariables	= sysd.GetVariablesList();

	double maxviolation = 0.;

	// --
	// Count active linear constraints..

	int n_c=0;
	int n_d=0;
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
	{
		if (mconstraints[ic]->IsActive())
		  if (mconstraints[ic]->IsLinear())
			if (mconstraints[ic]->IsUnilateral())
				n_d++;
			else
				n_c++;
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

	if (n_q==0) return 0;

	int n_docs = n_c + n_d;
	int n_vars = n_q + n_docs;
	
	// --
	// Reset and resize (if needed) auxiliary vectors

	MC->Reset(n_vars,n_vars);	// fast! Reset() method does not realloc if size doesn't change
	B->Reset(n_vars,1);
	X->Reset(n_vars,1);

	if (unilaterals != 0)
		{delete[]unilaterals; unilaterals = 0;}
	if (n_d > 0)
	{
		unilaterals = new ChUnilateralData[n_d];
	}



	// -- 
	// Fills the MC matrix and B vector, to pass to the sparse LCP simplex solver.
	// The original problem, stated as
	//  | M -Cq'|*|q|- | f|= |0| ,   c>=0, l>=0, l*c=0;
    //  | Cq  0 | |l|  |-b|  |c|
	// will be used with a small modification as:
	//  | M  Cq'|*| q|- | f|= |0| ,   c>=0, l>=0, l*c=0;
    //  | Cq  0 | |-l|  |-b|  |c|
	// so that it uses a symmetric MC matrix (the LDL factorization at each
	// pivot is happier:)

	// .. fills M submasses and 'f' part of B
	int s_q=0;
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
	{
		if (mvariables[iv]->IsActive())
		{
			mvariables[iv]->Build_M(*MC, s_q, s_q);				// .. fills  MC (M part)
			B->PasteMatrix(&mvariables[iv]->Get_fb(),s_q, 0);	// .. fills  B  (f part)
			if (add_Mq_to_f)
			{
				ChMatrixDynamic<double> mbv(	mvariables[iv]->Get_qb().GetRows(),
												mvariables[iv]->Get_qb().GetColumns() );
				mvariables[iv]->Compute_Mb_v(mbv, mvariables[iv]->Get_qb());
				B->PasteSumMatrix(&mbv, s_q,0);
			}
			s_q += mvariables[iv]->Get_ndof();
		}
	}  
	
	// .. fills M jacobians (only lower part) and 'b' part of B
	int s_c=0;
	int s_d=0;
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
	{
		if (mconstraints[ic]->IsActive())
		  if (mconstraints[ic]->IsLinear())
			if (mconstraints[ic]->IsUnilateral())
			{
				mconstraints[ic]->Build_Cq (*MC, n_q + n_c + s_d);// .. fills MC (Cq  part)
				mconstraints[ic]->Build_CqT(*MC, n_q + n_c + s_d);// .. fills MC (Cq' part)
				B->SetElement(n_q + n_c + s_d, 0, 
						-mconstraints[ic]->Get_b_i() );			// .. fills B  (c part)
						unilaterals[s_d].status = CONSTR_UNILATERAL_OFF;
				s_d++;
			}
			else
			{
				mconstraints[ic]->Build_Cq (*MC, n_q + s_c);
				mconstraints[ic]->Build_CqT(*MC, n_q + s_c);
				B->SetElement(n_q + s_c,       0,
						-mconstraints[ic]->Get_b_i() );
				s_c++;
			}
	}

	// -- 
	// Solve the LCP
 
	MC->SolveLCP(B, X,
				   n_c, n_d,
				   truncation_step, 
				   false,
				   unilaterals);

	// --
	// Update results into variable-interface objects
	s_q=0;
	for (unsigned int iv = 0; iv< mvariables.size(); iv++)
	{
		if (mvariables[iv]->IsActive())
		{
			mvariables[iv]->Get_qb().PasteClippedMatrix(X, 
						s_q,0, 
						mvariables[iv]->Get_ndof(),1, 
						0,0);
			s_q += mvariables[iv]->Get_ndof();
		}
	}
	 
	// --
	// Update results into constraint-interface objects 
	s_c=0;
	s_d=0;
	for (unsigned int ic = 0; ic< mconstraints.size(); ic++)
	{
		if (mconstraints[ic]->IsActive())
		  if (mconstraints[ic]->IsLinear())
			if (mconstraints[ic]->IsUnilateral())
			{   //(change sign of multipliers!)
				mconstraints[ic]->Set_l_i( -X->GetElement(n_q + n_c + s_d , 0));
				s_d++;
			}
			else
			{   //(change sign of multipliers!)
				mconstraints[ic]->Set_l_i( -X->GetElement(n_q + s_c ,  0));
				s_c++;
			}
	}

	return maxviolation;

}








} // END_OF_NAMESPACE____


