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

#ifndef CHLCPITERATIVEBB_H
#define CHLCPITERATIVEBB_H

//////////////////////////////////////////////////
//
//   ChLcpIterativeBB.h
//
//  An iterative solver based on modified 
//  Krylov iteration of spectral projected gradients
//  with Borzilai-Borwein 
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

/// An iterative solver based on modified 
/// Krylov iteration of spectral projected gradients
/// with Borzilai-Borwein.
/// The problem is described by a variational inequality VI(Z*x-d,K):
///
///  | M -Cq'|*|q|- | f|= |0| , l \in Y, C \in Ny, normal cone to Y  
///  | Cq -E | |l|  |-b|  |c|    
///
/// Also Z symmetric by flipping sign of l_i: |M  Cq'|*| q|-| f|=|0|  
///                                           |Cq  E | |-l| |-b| |c|
/// * case linear problem:  all Y_i = R, Ny=0, ex. all bilaterals)
/// * case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0)
/// * case CCP: Y_i are friction cones)

class ChApi ChLcpIterativeBB : public ChLcpIterativeSolver
{
protected:
			//
			// DATA
			//
	int		n_armijo;
	int		max_armijo_backtrace;
	bool	diag_preconditioning;

public:
			//
			// CONSTRUCTORS
			//

	ChLcpIterativeBB(
				int mmax_iters=50,      ///< max.number of iterations
				bool mwarm_start=false,	///< uses warm start?
				double mtolerance=0.0   ///< tolerance for termination criterion
				)  
			: ChLcpIterativeSolver(mmax_iters,mwarm_start, mtolerance,0.2)
			{
				n_armijo = 10;
				max_armijo_backtrace = 3;
				diag_preconditioning = true;
			};
				
	virtual ~ChLcpIterativeBB() {};

			//
			// FUNCTIONS
			//

				/// Performs the solution of the LCP.
				/// \return  the maximum constraint violation after termination.

	virtual double Solve(
				ChLcpSystemDescriptor& sysd		///< system description with constraints and variables	 
				);


				/// Same as Solve(), but this also supports the presence of
				/// ChLcpKblock blocks. If Solve() is called and stiffness is present,
				/// Solve() automatically falls back to this function.
				/// It does not solve the Schur complement N*l-r=0 as Solve does, here the 
				/// entire system KKT matrix with duals l and primals q is used.
				/// ***NOT WORKING***
	virtual double Solve_SupportingStiffness(
				ChLcpSystemDescriptor& sysd		///< system description with constraints and variables	
				);


				/// Number of max tolerated steps in non-monotone Armijo
				/// line search; usually good values are in 1..10 range.
	void   SetNarmijo (int mf) {this->n_armijo = mf;}
	double GetNarmijo () {return this->n_armijo;}
				
	void SetMaxArmijoBacktrace (int mm) {this->max_armijo_backtrace = mm;}
	int  GetMaxArmijoBacktrace () {return this->max_armijo_backtrace;}

				/// Enable diagonal preconditioning. It a simple but fast
				/// preconditioning technique that is expecially useful to 
				/// fix slow convergence in case variables have very different orders
				/// of magnitude.
	void SetDiagonalPreconditioning(bool mp) {this->diag_preconditioning = mp;}
	bool GetDiagonalPreconditioning() {return this->diag_preconditioning;}
};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpIterativeBB.h
