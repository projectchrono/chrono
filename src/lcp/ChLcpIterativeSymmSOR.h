#ifndef CHLCPITERATIVESYMMSOR_H
#define CHLCPITERATIVESYMMSOR_H

//////////////////////////////////////////////////
//
//   ChLcpIterativeSOR.h
//
//   An iterative LCP solver based on symmetric projective
//   fixed point method, with overrelaxation
//   and immediate variable update as in SSOR methods.
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


///    An iterative LCP solver based on symmetric projective
///   fixed point method, with overrelaxation
///   and immediate variable update as in SSOR methods.
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

class ChApi ChLcpIterativeSymmSOR : public ChLcpIterativeSolver
{
protected:
			//
			// DATA
			//


public:
			//
			// CONSTRUCTORS
			//

	ChLcpIterativeSymmSOR(
				int mmax_iters=50,      ///< max.number of iterations
				bool mwarm_start=false,	///< uses warm start?
				double mtolerance=0.0,  ///< tolerance for termination criterion
				double momega=1.0       ///< overrelaxation criterion
				)  
			: ChLcpIterativeSolver(mmax_iters,mwarm_start, mtolerance, momega) 
			{};
				
	virtual ~ChLcpIterativeSymmSOR() {};

			//
			// FUNCTIONS
			//

				/// Performs the solution of the LCP.
				/// \return  the maximum constraint violation after termination.

	virtual double Solve(
				ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
				bool add_Mq_to_f = false					///< if true, takes the initial 'q' and adds [M]*q to 'f' vector  
				);


				/// Set the overrelaxation factor, as in SOR methods. This
				/// factor may accelerate convergence if greater than 1. Optimal 
				/// value depends on the type of problem and it may be difficult 
				/// to extimate. 
				/// Default=1. Value clamped if less than 0.
	//void   SetOmega(double mval) {if (mval>0.) omega= mval;}
	//double GetOmega() {return omega;}

};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpIterativeSymmSOR.h
