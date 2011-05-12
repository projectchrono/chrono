#ifndef CHLCPSOLVERDEM_H
#define CHLCPSOLVERDEM_H

//////////////////////////////////////////////////
//
//   ChLcpSolverDEM.h
//
//  A solver for DEM type simulations, currently
//  without any support for bilateral constraints.
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


///    An iterative LCP solver based on projective
///   fixed point method, with overrelaxation
///   and immediate variable update as in SOR methods.
///    This solver must be used for mixed-linear
///   complementarity problems (MLCP) in this
///   form:
///
///    | M -Cq'|*|q|- | f|= |0| ,   c>=0, l>=0, l*c=0;
///    | Cq  0 | |l|  |-b|  |c|
///
///   as arising in the solution of QP with
///   inequalities or in multibody problems.
///    Note that this solver supports also the case of
///   boxed constraints on 'l', such as lmin<l<lmax

class ChApi ChLcpSolverDEM : public ChLcpDirectSolver
{
protected:
			//
			// DATA
			//


public:
			//
			// CONSTRUCTORS
			//

	ChLcpSolverDEM() : ChLcpDirectSolver()
			{};
				
	virtual ~ChLcpSolverDEM() {};

			//
			// FUNCTIONS
			//

				/// Performs the solution of the LCP.
				/// \return  the maximum constraint violation after termination.

	virtual double Solve(
				ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
				bool add_Mq_to_f = false					///< if true, takes the initial 'q' and adds [M]*q to 'f' vector  
				);



};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpIterativeSOR.h
