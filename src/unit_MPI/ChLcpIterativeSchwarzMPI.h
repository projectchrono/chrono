//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

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
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "unit_MPI/ChApiMPI.h"
#include "lcp/ChLcpIterativeSolver.h"


namespace chrono
{


/// This is an iterative LCP solver that can be used
/// when multiple processes are using ChLcpSystemDescriptorMPI
/// that can communicate via MPI. Hence, the solution
/// comes from a Schwarz alternating method.

class ChApiMPI ChLcpIterativeSchwarzMPI : public ChLcpIterativeSolver
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
				int mdomain_iters=10,	///< number of outer iterations (Schwarz inter-domain updates)
				int minner_iters=20,    ///< number of inner domain solver iterations
				bool mwarm_start=false,	///< uses warm start?
				double mtolerance=0.0,  ///< tolerance for termination criterion
				double momega=1.0      ///< overrelaxation criterion
				)  
			: ChLcpIterativeSolver(minner_iters,mwarm_start, mtolerance,momega) 
			{
				domain_iters = mdomain_iters;
			};
				
	virtual ~ChLcpIterativeSchwarzMPI() {};

			//
			// FUNCTIONS
			//

				/// Set outer (Schwarz) iterations, for domain updates. 
	void SetDomainIters(int niters) {domain_iters = niters;}
				/// Get outer (Schwarz) iterations, for domain updates.
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
