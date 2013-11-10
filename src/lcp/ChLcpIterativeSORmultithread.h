//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLCPITERATIVESORMULTITHREAD_H
#define CHLCPITERATIVESORMULTITHREAD_H

//////////////////////////////////////////////////
//
//   ChLcpIterativeSORmultithread.h
//
//  An iterative VI solver based on projective
//  fixed point method, with overrelaxation
//  and immediate variable update as in SOR methods,
//  implemented on multicore processors.
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpIterativeSolver.h"
#include "parallel/ChThreads.h"


namespace chrono
{


class ChApi ChLcpIterativeSORmultithread : public ChLcpIterativeSolver
{
protected:
			//
			// DATA
			//

	ChThreads* solver_threads;


public:
			//
			// CONSTRUCTORS
			//

	ChLcpIterativeSORmultithread(
				char* uniquename,		///< this name must be unique.
				int nthreads=2,			///< number of threads
				int mmax_iters=50,      ///< max.number of iterations
				bool mwarm_start=false,	///< uses warm start?
				double mtolerance=0.0,  ///< tolerance for termination criterion
				double momega=1.0       ///< overrelaxation criterion
				);  
				
	virtual ~ChLcpIterativeSORmultithread();

			//
			// FUNCTIONS
			//

				/// Performs the solution of the LCP.
				/// \return  the maximum constraint violation after termination.

	virtual double Solve(
				ChLcpSystemDescriptor& sysd		///< system description with constraints and variables
				);



				/// Changes the number of threads which run in parallel (should be > 1 )
	void ChangeNumberOfThreads(int mthreads = 2);
};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpIterativeSORmultithread.h
