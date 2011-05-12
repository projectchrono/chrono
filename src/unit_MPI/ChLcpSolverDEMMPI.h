#ifndef CHLCPSOLVERDEMMPI_H
#define CHLCPSOLVERDEMMPI_H

//////////////////////////////////////////////////
//
//   ChLcpSolverDEMMPI.h
//
//  A solver that operates on
//  multiple domains. Requires the ChLcpSystemDescriptorMPI ! 
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "unit_MPI/ChApiMPI.h"
#include "lcp/ChLcpDirectSolver.h"


namespace chrono
{


/// This is a solver that can be used
/// when multiple processes are using ChLcpSystemDescriptorMPI
/// that can communicate via MPI.

class ChApiMPI ChLcpSolverDEMMPI : public ChLcpDirectSolver
{
protected:
			//
			// DATA
			//


public:
			//
			// CONSTRUCTORS
			//

	ChLcpSolverDEMMPI() : ChLcpDirectSolver() 
			{};
				
	virtual ~ChLcpSolverDEMMPI() {};

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




#endif  // END of ChLcpSolverDEMMPI.h
