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

#ifndef CHLCPITERATIVEJACOBI_H
#define CHLCPITERATIVEJACOBI_H

//////////////////////////////////////////////////
//
//   ChLcpIterativeJacobi.h
//
//  An iterative LCP solver based on projective
//  fixed point method, similar to projected Jacobi.
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpIterativeSolver.h"


namespace chrono
{


/// An iterative solver for VI (VI/CCP/LCP/linear problems,..) based 
/// on projective fixed point method, similar to a projected 
/// Jacobi method.
/// Note: this method is here mostly for comparison
/// and tests: we suggest you to use the more efficient
/// ChLcpIterativeSOR - similar, but faster & converges better.
/// The problem is described by a variational inequality VI(Z*x-d,K):
///
///  | M -Cq'|*|q|- | f|= |0| , l \in Y, c \in Ny, normal cone to Y  
///  | Cq -E | |l|  |-b|  |c|    
///
/// Also Z symmetric by flipping sign of l_i: |M  Cq'|*| q|-| f|=|0|  
///                                           |Cq  E | |-l| |-b| |c|
/// * case linear problem:  all Y_i = R, Ny=0, ex. all bilaterals
/// * case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0
/// * case CCP: Y_i are friction cones

class ChApi ChLcpIterativeJacobi : public ChLcpIterativeSolver
{
protected:
			//
			// DATA
			//


public:
			//
			// CONSTRUCTORS
			//

	ChLcpIterativeJacobi(
				int mmax_iters=50,      ///< max.number of iterations
				bool mwarm_start=false,	///< uses warm start?
				double mtolerance=0.0,  ///< tolerance for termination criterion
				double momega=0.2       ///< overrelaxation criterion
				)  
			: ChLcpIterativeSolver(mmax_iters,mwarm_start, mtolerance, momega)
			{};
				
	virtual ~ChLcpIterativeJacobi() {};

			//
			// FUNCTIONS
			//

				/// Performs the solution of the LCP.
				/// You must provide a ChLcpSystemDescriptor.
				/// \return  the maximum constraint violation after termination.

	virtual double Solve(
				ChLcpSystemDescriptor& sysd		///< system description with constraints and variables		 
				);

};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpIterativeJacobi.h
