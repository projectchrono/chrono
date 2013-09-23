//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLCPITERATIVEMINRES_H
#define CHLCPITERATIVEMINRES_H

//////////////////////////////////////////////////
//
//   ChLcpIterativeMINRES.h
//
//  An iterative LCP solver based on modified 
//  Krylov iteration of MINRES type alternated with
//  gradeint projection.
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
/// Krylov iteration of MINRES type alternated with
/// gradient projection (active set)
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

class ChApi ChLcpIterativeMINRES : public ChLcpIterativeSolver
{
protected:
			//
			// DATA
			//
	double	feas_tolerance;
	int		max_fixedpoint_steps;

public:
			//
			// CONSTRUCTORS
			//

	ChLcpIterativeMINRES(
				int mmax_iters=50,      ///< max.number of iterations
				bool mwarm_start=false,	///< uses warm start?
				double mtolerance=0.0   ///< tolerance for termination criterion
				)  
			: ChLcpIterativeSolver(mmax_iters,mwarm_start, mtolerance,0.2)
			{
				feas_tolerance = 0.2;
				max_fixedpoint_steps = 6;
			};
				
	virtual ~ChLcpIterativeMINRES() {};

			//
			// FUNCTIONS
			//

				/// Performs the solution of the LCP.
				/// \return  the maximum constraint violation after termination.

	virtual double Solve(
				ChLcpSystemDescriptor& sysd,		///< system description with constraints and variables	
				bool add_Mq_to_f = false			///< if true, takes the initial 'q' and adds [M]*q to 'f' vector  
				);


	void   SetFeasTolerance (double mf) {this->feas_tolerance = mf;}
	double GetFeasTolerance () {return this->feas_tolerance;}

	void SetMaxFixedpointSteps (int mm) {this->max_fixedpoint_steps = mm;}
	int  GetMaxFixedpointSteps () {return this->max_fixedpoint_steps;}


};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpIterativeMINRES.h
