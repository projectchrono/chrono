#ifndef CHLCPITERATIVEMINRES_H
#define CHLCPITERATIVEMINRES_H

//////////////////////////////////////////////////
//
//   ChLcpIterativeMINRES.h
//
//  An iterative LCP solver based on modified 
//  Krylov iteration of MINRES type alternated with
//  gradeint projection.
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
/// Krylov iteration of MINRES type alternated with
/// gradient projection.
/// The problem is described by an LCP of type
///
///    | M -Cq'|*|q|- | f|= |0| ,   c>=0, l>=0, l*c=0;
///    | Cq  0 | |l|  |-b|  |c|
///
/// or similar CCP problem.

class ChLcpIterativeMINRES : public ChLcpIterativeSolver
{
protected:
			//
			// DATA
			//
	double	feas_tolerance;
	int		max_fixedpoint_steps;

public:
			//
			// CONSTRUCTORS
			//

	ChLcpIterativeMINRES(
				int mmax_iters=50,      ///< max.number of iterations
				bool mwarm_start=false,	///< uses warm start?
				double mtolerance=0.0   ///< tolerance for termination criterion
				)  
			: ChLcpIterativeSolver(mmax_iters,mwarm_start, mtolerance,0.2)
			{
				feas_tolerance = 0.2;
				max_fixedpoint_steps = 6;
			};
				
	virtual ~ChLcpIterativeMINRES() {};

			//
			// FUNCTIONS
			//

				/// Performs the solution of the LCP.
				/// \return  the maximum constraint violation after termination.

	virtual double Solve(
				ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
				bool add_Mq_to_f = false			///< if true, takes the initial 'q' and adds [M]*q to 'f' vector  
				);


	void   SetFeasTolerance (double mf) {this->feas_tolerance = mf;}
	double GetFeasTolerance () {return this->feas_tolerance;}

	void SetMaxFixedpointSteps (int mm) {this->max_fixedpoint_steps = mm;}
	int  GetMaxFixedpointSteps () {return this->max_fixedpoint_steps;}


};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpIterativeMINRES.h
