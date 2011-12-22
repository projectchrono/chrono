#ifndef CHLCPITERATIVEPMINRES_H
#define CHLCPITERATIVEPMINRES_H

//////////////////////////////////////////////////
//
//   ChLcpIterativePMINRES.h
//
//  An iterative LCP solver based on modified 
//  Krylov iteration of MINRES type with gradient
//  projections.
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

/// An iterative LCP solver based on modified 
/// Krylov iteration of MINRES type with gradient 
/// projections (similar to nonlinear CG with Polyak-Ribiere)
/// The problem is described by an LCP of type
///
///    | M -Cq'|*|q|- | f|= |0| ,   c>=0, l>=0, l*c=0;
///    | Cq  0 | |l|  |-b|  |c|
///
/// or similar CCP problem.

class ChApi ChLcpIterativePMINRES : public ChLcpIterativeSolver
{
protected:
			//
			// DATA
			//
	double	grad_diffstep;

public:
			//
			// CONSTRUCTORS
			//

	ChLcpIterativePMINRES(
				int mmax_iters=50,      ///< max.number of iterations
				bool mwarm_start=false,	///< uses warm start?
				double mtolerance=0.0   ///< tolerance for termination criterion
				)  
			: ChLcpIterativeSolver(mmax_iters,mwarm_start, mtolerance,0.2)
			{
				grad_diffstep = 0.0001;
			};
				
	virtual ~ChLcpIterativePMINRES() {};

			//
			// FUNCTIONS
			//

				/// Performs the solution of the LCP.
				/// \return  the maximum constraint violation after termination.

	virtual double Solve(
				ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
				bool add_Mq_to_f = false			///< if true, takes the initial 'q' and adds [M]*q to 'f' vector  
				);


	void   SetGradDiffStep (double mf) {this->grad_diffstep = mf;}
	double GetGradDiffStep () {return this->grad_diffstep;}


};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpIterativePMINRES.h
