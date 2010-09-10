#ifndef CHLCPITERATIVESCHWARZMPI_H
#define CHLCPITERATIVESCHWARZMPI_H

//////////////////////////////////////////////////
//
//   ChLcpIterativeSchwarzMPI.h
//
//  An iterative LCP solver that operates on
//  multiple domains. Requires the ChLcpSystemDescriptorMPI ! 
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "lcp/ChLcpIterativeSolver.h"


namespace chrono
{


/// This is an iterative LCP solver that can be used
/// when multiple processes are using ChLcpSystemDescriptorMPI
/// that can communicate via MPI. Hence, the solution
/// comes from a Schwarz alternating method.

class ChLcpIterativeSchwarzMPI : public ChLcpIterativeSolver
{
protected:
			//
			// DATA
			//

	int domain_iters;

public:
			//
			// CONSTRUCTORS
			//

	ChLcpIterativeSchwarzMPI(
				int mmax_iters=60,      ///< max.number of iterations (counted as tot. inner solver iters)
				bool mwarm_start=false,	///< uses warm start?
				double mtolerance=0.0,  ///< tolerance for termination criterion
				double momega=1.0,      ///< overrelaxation criterion
				int mdomain_iters=10	///< performs an inter-domain schwarz update each domain_iters iters 	
				)  
			: ChLcpIterativeSolver(mmax_iters,mwarm_start, mtolerance,momega) 
			{
				domain_iters = mdomain_iters;
			};
				
	virtual ~ChLcpIterativeSchwarzMPI() {};

			//
			// FUNCTIONS
			//

				/// Performs an inter-domain Schwarz update each n iters, that you can
				/// can set here. For instance, if you set 4, if 'o' represents the
				/// inner domain solver iteration and '|' represents the Schwarz update,
				/// the pattern will be oooo|oooo|oooo|... until end of mmax_iters.
	void SetDomainIters(int niters) {domain_iters = niters;}
	int  GetDomainIters() {return domain_iters;}

				/// Performs the solution of the LCP.
				/// \return  the maximum constraint violation after termination.

	virtual double Solve(
				ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
				bool add_Mq_to_f = false					///< if true, takes the initial 'q' and adds [M]*q to 'f' vector  
				);



};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpIterativeSchwarzMPI.h
