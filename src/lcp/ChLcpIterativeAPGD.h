#ifndef CHLCPITERATIVEAPGD_H
#define CHLCPITERATIVEAPGD_H

//////////////////////////////////////////////////
//
//   ChLcpIterativeAPGD.h
//
//  An iterative solver based on Nesterov's
//  Projected Gradient Descent
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpIterativeSolver.h"


namespace chrono
{

/// An iterative solver based on Nesterov's
/// Projected Gradient Descent.
/// The problem is described by an LCP of type
///
///    | M -Cq'|*|q|- | f|= |0| ,   c>=0, l>=0, l*c=0;
///    | Cq  0 | |l|  |-b|  |c|
///
/// or similar CCP problem.

class ChApi ChLcpIterativeAPGD : public ChLcpIterativeSolver
{
protected:
			//
			// DATA
			//

public:
			//
			// CONSTRUCTORS
			//

	ChLcpIterativeAPGD(
				int mmax_iters=50,      ///< max.number of iterations
				bool mwarm_start=false,	///< uses warm start?
				double mtolerance=0.0   ///< tolerance for termination criterion
				)  
			: ChLcpIterativeSolver(mmax_iters,mwarm_start, mtolerance,0.2)
			{
				
			};
				
	virtual ~ChLcpIterativeAPGD() {};

			//
			// FUNCTIONS
			//

				/// Performs the solution of the LCP.
				/// \return  the maximum constraint violation after termination.

	virtual double Solve(
				ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
				bool add_Mq_to_f = false			///< if true, takes the initial 'q' and adds [M]*q to 'f' vector  
				);


};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpIterativeAPGD.h
