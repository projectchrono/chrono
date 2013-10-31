//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

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
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "unit_MPI/ChApiMPI.h"
#include "lcp/ChLcpIterativeSolver.h"


namespace chrono
{


/// This is a solver that can be used
/// when multiple processes are using ChLcpSystemDescriptorMPI
/// that can communicate via MPI.

class ChApiMPI ChLcpSolverDEMMPI : public ChLcpIterativeSolver
{
protected:
			//
			// DATA
			//


public:
			//
			// CONSTRUCTORS
			//

	ChLcpSolverDEMMPI(
							int mmax_iters=50,      ///< max.number of iterations
							bool mwarm_start=false,	///< uses warm start?
							double mtolerance=0.0,  ///< tolerance for termination criterion
							double momega=1.0       ///< overrelaxation criterion
							)
			: ChLcpIterativeSolver(mmax_iters,mwarm_start, mtolerance,momega)
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
