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

class ChApi ChLcpSolver
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



};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpSolver.h
