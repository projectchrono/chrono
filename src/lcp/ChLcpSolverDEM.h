//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLCPSOLVERDEM_H
#define CHLCPSOLVERDEM_H

//////////////////////////////////////////////////
//
//   ChLcpSolverDEM.h
//
//   A solver for DEM type simulations.
//
///////////////////////////////////////////////////


#include "ChLcpIterativeSolver.h"


namespace chrono
{


/// A penalty-based solver for complementarity problem as arising in DEM formulations.
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
public:
	ChLcpSolverDEM(int    mmax_iters  = 50,     ///< max.number of iterations
	               bool   mwarm_start = false,  ///< uses warm start?
	               double mtolerance  = 0.0,    ///< tolerance for termination criterion
	               double momega      = 1.0)    ///< overrelaxation criterion
	: ChLcpIterativeSolver(mmax_iters, mwarm_start, mtolerance, momega)
	{}

	~ChLcpSolverDEM() {}

	/// Performs the solution of the LCP.
	/// \return  the maximum constraint violation after termination.
	virtual double Solve(ChLcpSystemDescriptor& sysd);
};



} // END_OF_NAMESPACE____



#endif
