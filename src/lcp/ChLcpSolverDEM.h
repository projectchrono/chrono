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



#include "ChLcpIterativeSolver.h"


namespace chrono
{


///  A penalty-based solver for complementarity problem
/// as arising in DEM formulations.
/// The problem is described by a variational inequality VI(Z*x-d,K):
///
///  | M -Cq'|*|q|- | f|= |0| , l \in Y, C \in Ny, normal cone to Y  
///  | Cq -E | |l|  |-b|  |c|    
///
/// Also Z symmetric by flipping sign of l_i: |M  Cq'|*| q|-| f|=|0|  
///                                           |Cq  E | |-l| |-b| |c|
/// * case linear problem:  all Y_i = R, Ny=0, ex. all bilaterals
/// * case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0
/// * case CCP: Y_i are friction cones

class ChApi ChLcpSolverDEM : public ChLcpIterativeSolver
{
protected:
			//
			// DATA
			//


public:
			//
			// CONSTRUCTORS
			//

	ChLcpSolverDEM(
					int mmax_iters=50,      ///< max.number of iterations
					bool mwarm_start=false,	///< uses warm start?
					double mtolerance=0.0,  ///< tolerance for termination criterion
					double momega=1.0       ///< overrelaxation criterion
					)
		: ChLcpIterativeSolver(mmax_iters,mwarm_start, mtolerance,momega)
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
