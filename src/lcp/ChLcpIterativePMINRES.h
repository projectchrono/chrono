//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLCPITERATIVEPMINRES_H
#define CHLCPITERATIVEPMINRES_H

//////////////////////////////////////////////////
//
//   ChLcpIterativePMINRES.h
//
//  An iterative LCP solver based on modified 
//  Krylov iteration of MINRES type with gradient
//  projections.
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

/// An iterative LCP solver based on modified 
/// Krylov iteration of MINRES type with gradient 
/// projections (similar to nonlinear CG with Polyak-Ribiere)
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

class ChApi ChLcpIterativePMINRES : public ChLcpIterativeSolver
{
protected:
			//
			// DATA
			//
	double	grad_diffstep;
	double  rel_tolerance;
	bool    diag_preconditioning;

public:
			//
			// CONSTRUCTORS
			//

	ChLcpIterativePMINRES(
				int mmax_iters=50,      ///< max.number of iterations
				bool mwarm_start=false,	///< uses warm start?
				double mtolerance=0.0   ///< tolerance for termination criterion
				)  
			: ChLcpIterativeSolver(mmax_iters,mwarm_start, mtolerance,0.2)
			{
				grad_diffstep = 0.01; // too small can cause numerical roundoff troubles!
				rel_tolerance = 0.0;
				diag_preconditioning = true;
			};
				
	virtual ~ChLcpIterativePMINRES() {};

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
	virtual double Solve_SupportingStiffness(
				ChLcpSystemDescriptor& sysd		///< system description with constraints and variables	 
				);

				/// For the case where inequalities are introduced, the 
				/// gradient is projected. A numerical differentiation is used, this is the delta. 
	void   SetGradDiffStep (double mf) {this->grad_diffstep = mf;}
	double GetGradDiffStep () {return this->grad_diffstep;}

				/// Set relative tolerance. the iteration stops when
				/// the (projected) residual r is smaller than absolute tolerance,
				/// that you set via SetTolerance(), OR it is smaller than 'rhs_rel_tol' i.e. the norm
				/// of the right hand side multiplied by this relative tolerance. 
				/// Set to 0 if you do not want relative tolerance to enter into play.
	void   SetRelTolerance (double mrt) {this->rel_tolerance = mrt;}
	double GetRelTolerance () {return this->rel_tolerance;}

				/// Enable diagonal preconditioning. It a simple but fast
				/// preconditioning technique that is expecially useful to 
				/// fix slow convergence in case variables have very different orders
				/// of magnitude.
	void SetDiagonalPreconditioning(bool mp) {this->diag_preconditioning = mp;}
	bool GetDiagonalPreconditioning() {return this->diag_preconditioning;}

};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpIterativePMINRES.h
