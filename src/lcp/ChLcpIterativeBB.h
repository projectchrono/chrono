#ifndef CHLCPITERATIVEBB_H
#define CHLCPITERATIVEBB_H

//////////////////////////////////////////////////
//
//   ChLcpIterativeBB.h
//
//  An iterative solver based on modified 
//  Krylov iteration of spectral projected gradients
//  with Borzilai-Borwein 
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

/// An iterative solver based on modified 
/// Krylov iteration of spectral projected gradients
/// with Borzilai-Borwein.
/// The problem is described by an LCP of type
///
///    | M -Cq'|*|q|- | f|= |0| ,   c>=0, l>=0, l*c=0;
///    | Cq  0 | |l|  |-b|  |c|
///
/// or similar CCP problem.

class ChApi ChLcpIterativeBB : public ChLcpIterativeSolver
{
protected:
			//
			// DATA
			//
	int		n_armijo;
	int		max_armijo_backtrace;

public:
			//
			// CONSTRUCTORS
			//

	ChLcpIterativeBB(
				int mmax_iters=50,      ///< max.number of iterations
				bool mwarm_start=false,	///< uses warm start?
				double mtolerance=0.0   ///< tolerance for termination criterion
				)  
			: ChLcpIterativeSolver(mmax_iters,mwarm_start, mtolerance,0.2)
			{
				n_armijo = 10;
				max_armijo_backtrace = 15;
			};
				
	virtual ~ChLcpIterativeBB() {};

			//
			// FUNCTIONS
			//

				/// Performs the solution of the LCP.
				/// \return  the maximum constraint violation after termination.

	virtual double Solve(
				ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
				bool add_Mq_to_f = false			///< if true, takes the initial 'q' and adds [M]*q to 'f' vector  
				);


	void   SetNarmijo (int mf) {this->n_armijo = mf;}
	double GetNarmijo () {return this->n_armijo;}

	void SetMaxArmijoBacktrace (int mm) {this->max_armijo_backtrace = mm;}
	int  GetMaxArmijoBacktrace () {return this->max_armijo_backtrace;}


};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpIterativeBB.h
