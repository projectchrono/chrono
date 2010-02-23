#ifndef CHLCPSOLVER_H
#define CHLCPSOLVER_H

//////////////////////////////////////////////////
//
//   ChLcpSolver.h
//
//    Base class for all solvers aimed at solving
//   LCP linear complementarity problems, as arising
//   from QP optimization problems.
//
//    Inherited solvers will solve mixed-linear
//   complementarity problems (MLCP) in this
//   form:
//
//    | M -Cq'|*|q|- | f|= |0| ,   c>=0, l>=0, l*c=0;
//    | Cq  0 | |l|  |-b|  |c|
//
//   as arising in the solution of QP with
//   inequalities or in multibody problems.
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <vector>
#include "ChLcpConstraint.h"
#include "ChLcpVariables.h"
#include "ChLcpSystemDescriptor.h"

namespace chrono
{

///    Base class for solvers aimed at solving
///   LCP linear complementarity problems arising
///   from QP optimization problems.
///    This class does nothing: it is up to inherited
///   classes to implement specific solution methods,
///   such as simplex, iterative SOR, etc.
///    The LCP problem must be in this (symmetric) 
///   form:
///
///    | M -Cq'|*|q|- | f|= |0| ,   c>=0, l>=0, l*c=0;
///    | Cq  0 | |l|  |-b|  |c|
///
///   as arising in the solution of QP with
///   inequalities or in multibody problems.

class ChLcpSolver
{
public:
			//
			// DATA
			//

			//
			// CONSTRUCTORS
			//

	ChLcpSolver() {};

	virtual ~ChLcpSolver() {};

			//
			// FUNCTIONS
			//

			// --Following functions are generic interfaces to the LCP solver. The
			//   Solve() function is a pure virtual method, so it MUST be implemented
			//   by specialized child classes:

				/// Performs the solution of the LCP.
				/// You must provide a system description using ChLcpSystemDescriptor.
				/// This function MUST be implemented in children classes, with specialized 
				/// methods such as iterative schemes, simplex schemes, fixed point algorithms, etc.
				/// \return  the maximum constraint violation after termination.

	virtual double Solve(
				ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
				bool add_Mq_to_f = false			///< if true, takes the initial 'q' and adds [M]*q to 'f' vector  
				) = 0;


			//
			// Utility functions
			//

				/// The following function may be called after the 'Solve()'
				/// operation has been performed. This gives an extimate of 'how
				/// good' the solver had been in finding the proper solution.
				/// Resulting estimates are passed as references in member arguments.

	virtual void ComputeFeasabilityViolation(
				std::vector<ChLcpConstraint*>& mconstraints,///< array of pointers to constraints
				double& resulting_maxviolation,		///< gets the max constraint violation (either bi- and unilateral.)
				double& resulting_lcpfeasability	///< gets the max feasability as max |l*c| , for unilateral only
				);

				/// Using this function, one may get a vector with all the variables 'q'
				/// ordered into a column vector. The column vector must be passed as a ChMatrix<>
				/// object, which will be automatically reset and resized to the proper length.
				/// \return  the number of scalar variables (i.e. the rows of the column vector).
	virtual int FromVariablesToVector(
								std::vector<ChLcpVariables*>& mvariables,	///< array of pointers to items with subparts of variables 'q'
								ChMatrix<>& mvector						///< matrix which will contain the entire vector of 'q'
								);

				/// Using this function, one may go in the opposite direction of the FromVariablesToVector()
				/// function, i.e. one gives a vector with all the variables 'q' ordered into a column vector, and
				/// the variables objects are updated according to these values.
				/// NOTE!!! differently from  FromVariablesToVector(), which always works, this
				/// function will fail if mvector does not match the amount and ordering of
				/// the variable objects!!! (it is up to the user to check this!) btw: most often,
				/// this is called after FromVariablesToVector() to do a kind of 'undo', for example.
				/// \return  the number of scalar variables (i.e. the rows of the column vector).
	virtual int FromVectorToVariables(
								ChMatrix<>& mvector,					///< matrix which will contain the entire vector of 'q'
								std::vector<ChLcpVariables*>& mvariables	///< array of pointers to items with subparts of variables 'q'
								);

};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpSolver.h
