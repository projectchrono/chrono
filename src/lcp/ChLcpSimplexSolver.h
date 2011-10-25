#ifndef CHLCPSIMPLEXSOLVER_H
#define CHLCPSIMPLEXSOLVER_H

//////////////////////////////////////////////////
//
//   ChLcpSimplexSolver.h
//
//     ***OBSOLETE****
//
//    A simplex (pivoting) method which solves LCP
//   problems by activating/deactivating constraints
//   and solving a linear problem each time (using a
//   custom sparse solver)
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpDirectSolver.h"



namespace chrono
{

// forward reference
class ChSparseMatrix;
class ChUnilateralData;

///     ***OBSOLETE****
///    A simplex (pivoting) method which solves LCP
///   problems by activating/deactivating constraints
///   and solving a linear problem each time (using a
///   custom sparse solver)
///    It is by far slower than iterative methods, but
///   it gives an almost exact solution (within numerical 
///   roundoff and accumulation errors, of course).
///    It can handle redundant constraints.
///    This solver must be used for mixed-linear
///   complementarity problems (MLCP) in this
///   form:
///
///    | M -Cq'|*|q|- | f|= |0| ,   c>=0, l>=0, l*c=0;
///    | Cq  0 | |l|  |-b|  |c|
///
///   as arising in the solution of QP with
///   inequalities or in multibody problems.


class ChApi ChLcpSimplexSolver : public ChLcpDirectSolver
{
protected:
			//
			// DATA
			//

	int truncation_step; // if 0 no effect, if >0 steps are truncated 
	ChSparseMatrix* MC;  // the sparse matrix for direct solution [MC]X=B (auto fill)
	ChMatrix<>* X;		 // the unknown vector (automatically filled) 
	ChMatrix<>* B;		 // the known vector (automatically filled)
	ChUnilateralData* unilaterals; // array with temporary info for pivoting

public:
			//
			// CONSTRUCTORS
			//

	ChLcpSimplexSolver();
				
	virtual ~ChLcpSimplexSolver();


			//
			// FUNCTIONS
			//

				/// Performs the solution of the LCP, using the simplex method.
				///  If you must solve many LCP problems with the same amount of
				/// variables and constraints, we suggest you to use the same 
				/// ChLcpSimplexSolver object, because it exploits coherency: avoids
				/// reallocating the sparse matrix each time.
				/// \return  the maximum constraint violation after termination.

	virtual double Solve(
				ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
				bool add_Mq_to_f = false			///< if true, takes the initial 'q' and adds [M]*q to 'f' vector  
				);

				/// Set truncation step (that is, the method stops anyway after
				/// this amount of pivots in simplex explorations, even if the 
				/// solution isn't reached). NOTE!! This limits cases with exponential 
				/// complexity explosion, but premature termination can give _meaningless_
				/// results (differently from premature termination of iterative methods).
				/// For truncation step = 0 continue endlessly up to exact solution 
				/// (default) 
	void   SetTruncationStep(int mstep) {truncation_step= mstep;}
	void   SetNoTruncation() {truncation_step=0;}
	double GetTruncationStep() {return truncation_step;}

};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpSimplexSolver.h
