#ifndef CHLCPITERATIVEJACOBI_H
#define CHLCPITERATIVEJACOBI_H

//////////////////////////////////////////////////
//
//   ChLcpIterativeJacobi.h
//
//  An iterative LCP solver based on projective
//  fixed point method, similar to projected Jacobi.
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


///    An iterative LCP solver based on projective
///   fixed point method, similar to a projected 
///   Jacobi method.
///   Note: this method is here mostly for comparison
///   and tests: we suggest you to use the more efficient
///   ChLcpIterativeSOR - similar, but faster & converges better.
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

class ChLcpIterativeJacobi : public ChLcpIterativeSolver
{
protected:
			//
			// DATA
			//


public:
			//
			// CONSTRUCTORS
			//

	ChLcpIterativeJacobi(
				int mmax_iters=50,      ///< max.number of iterations
				bool mwarm_start=false,	///< uses warm start?
				double mtolerance=0.0,  ///< tolerance for termination criterion
				double momega=0.2       ///< overrelaxation criterion
				)  
			: ChLcpIterativeSolver(mmax_iters,mwarm_start, mtolerance, momega)
			{};
				
	virtual ~ChLcpIterativeJacobi() {};

			//
			// FUNCTIONS
			//

				/// Performs the solution of the LCP.
				/// You must provide a ChLcpSystemDescriptor.
				/// \return  the maximum constraint violation after termination.

	virtual double Solve(
				ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables		
				bool add_Mq_to_f 					///< if true, takes the initial 'q' and adds [M]*q to 'f' vector  
				);

};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpIterativeJacobi.h
