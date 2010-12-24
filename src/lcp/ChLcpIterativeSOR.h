#ifndef CHLCPITERATIVESOR_H
#define CHLCPITERATIVESOR_H

//////////////////////////////////////////////////
//
//   ChLcpIterativeSOR.h
//
//  An iterative LCP solver based on projective
//  fixed point method, with overrelaxation
//  and immediate variable update as in SOR methods.
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

class ChApi ChLcpIterativeSOR : public ChLcpIterativeSolver
{
protected:
			//
			// DATA
			//


public:
			//
			// CONSTRUCTORS
			//

	ChLcpIterativeSOR(
				int mmax_iters=50,      ///< max.number of iterations
				bool mwarm_start=false,	///< uses warm start?
				double mtolerance=0.0,  ///< tolerance for termination criterion
				double momega=1.0       ///< overrelaxation criterion
				)  
			: ChLcpIterativeSolver(mmax_iters,mwarm_start, mtolerance,momega)
			{};
				
	virtual ~ChLcpIterativeSOR() {};

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
