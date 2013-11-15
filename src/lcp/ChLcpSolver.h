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

#ifndef CHLCPSOLVER_H
#define CHLCPSOLVER_H

//////////////////////////////////////////////////
//
//   ChLcpSolver.h
//
//    Base class for all solvers aimed at solving
//   LCP linear complementarity problems, as arising
//   from QP optimization problems.
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <vector>
#include "ChLcpConstraint.h"
#include "ChLcpVariables.h"
#include "ChLcpSystemDescriptor.h"

namespace chrono
{

///  Base class for solvers aimed at solving
/// LCP linear complementarity problems arising
/// from QP optimization problems.
///  This class does nothing: it is up to inherited
/// classes to implement specific solution methods,
/// such as simplex, iterative SOR, etc.
///  The problem is described by a variational inequality VI(Z*x-d,K):
///
///  | M -Cq'|*|q|- | f|= |0| , l \in Y, C \in Ny, normal cone to Y  
///  | Cq -E | |l|  |-b|  |c|    
///
/// Also Z symmetric by flipping sign of l_i: |M  Cq'|*| q|-| f|=|0|  
///                                           |Cq  E | |-l| |-b| |c|
/// * case linear problem:  all Y_i = R, Ny=0, ex. all bilaterals
/// * case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0
/// * case CCP: Y_i are friction cones

class ChApi ChLcpSolver
{
public:
			//
			// DATA
			//

	bool verbose;

			//
			// CONSTRUCTORS
			//

	ChLcpSolver() { verbose = false; };

	virtual ~ChLcpSolver() {};

			//
			// FUNCTIONS
			//

			// --Following functions are generic interfaces to the LCP solver. The
			//   Solve() function is a pure virtual method, so it MUST be implemented
			//   by specialized child classes:

				/// Performs the solution of the LCP.
				/// You must provide a system description using ChLcpSystemDescriptor.
				/// This function MUST be implemented in children classes, with specialized 
				/// methods such as iterative schemes, simplex schemes, fixed point algorithms, etc.
				/// \return  the maximum constraint violation after termination.

	virtual double Solve(
				ChLcpSystemDescriptor& sysd		///< system description with constraints and variables	 
				) = 0;


			//
			// Utility functions
			//

	void SetVerbose(bool mv) {this->verbose = mv;}
	bool GetVerbose() {return this->verbose;}

};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpSolver.h
