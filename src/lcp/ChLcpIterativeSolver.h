//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010, 2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLCPITERATIVESOLVER_H
#define CHLCPITERATIVESOLVER_H

//////////////////////////////////////////////////
//
//   ChLcpIterativeSolver.h
//
//    Base class for all solution methods
//   based on iterative schemes, such as the
//   SOR method and so on.
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLcpSolver.h"


namespace chrono
{


/// Base class for ITERATIVE solvers aimed at solving
/// LCP linear complementarity problems arising
/// from QP optimization problems.
/// This class does nothing: it is up to inherited
/// classes to implement specific solution methods,
/// such as simplex, iterative SOR, etc.
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


class ChApi ChLcpIterativeSolver : public ChLcpSolver
{
protected:
			//
			// DATA
			//

	int		max_iterations;
	int		tot_iterations;		///The total number of iterations performed by the solver
	bool	warm_start;
	double	tolerance;	
	double  omega;
	double  shlambda;

	bool	record_violation_history;
	std::vector<double> violation_history;
	std::vector<double>	dlambda_history;

public:
			//
			// CONSTRUCTORS
			//

	ChLcpIterativeSolver(
				int    mmax_iters=50,		///< max.number of iterations
				bool   mwarm_start=false,	///< uses warm start?
				double mtolerance=0.0,		///< tolerance for termination criterion
				double momega=1.0,			///< overrelaxation, if any
				double mshlambda=1.0		///< sharpness, if any 
				)
			: max_iterations(mmax_iters), 
			  tot_iterations(0),
			  warm_start(mwarm_start), 
			  tolerance(mtolerance),
			  omega(momega),
			  shlambda(mshlambda),
              record_violation_history(false)
			{
				violation_history.clear();
				dlambda_history.clear();
			};

	virtual ~ChLcpIterativeSolver() {};


			//
			// FUNCTIONS
			//

				/// Set the maximum number of iterations (if the 
				/// solver exceed this limit, it should stop even if
				/// the required tolerance isn't yet reached.
				/// Default limit: 50 iterations.
	virtual void   SetMaxIterations(int mval) {max_iterations=mval;}
	virtual double GetMaxIterations() {return max_iterations;}
	virtual double GetTotalIterations() {return tot_iterations;}

				/// Set the overrelaxation factor, if used; it can be 
				/// used by SOR-like methods.  
				/// Default=1. Value clamped if lower than 0. 
	virtual void   SetOmega(double mval) {if (mval>0.) omega= mval;}
	virtual double GetOmega() {return omega;}

				/// Set the sharpness factor, if used; it can be 
				/// used by SOR-like methods with projection (see Mangasarian LCP method)
				/// Default=1. Usual range in 0..1. Value clamped if lower than 0. 
	virtual void   SetSharpnessLambda(double mval) {if (mval>0.) shlambda= mval;}
	virtual double GetSharpnessLambda() {return shlambda;}

				///	Set if the solver should 'warm start' from the
				/// actual values of the variables. Useful if the
				/// variables are already near to the solution. In 
				/// most iterative schemes, this is often a good trick.
	virtual void   SetWarmStart(bool mval) {warm_start = mval;}
	virtual bool   GetWarmStart() {return warm_start;}
	
				/// Set the tolerance (default is 0.) for stopping
				/// criterion. The iteration is stopped when the 
				/// constraint feasability error goes under this 
				/// tolerance.
	virtual void   SetTolerance(double mval) {tolerance = mval;}
	virtual double GetTolerance() {return tolerance;}

				/// Set 'true' if you want the iterative solver to 
				/// record the values of constraint violation into 
				/// a vector with the history (for tuning and debugging,
				/// so you can plot a graph of how the residual decreases
				/// during iterations until solution is reached)
	void   SetRecordViolation(bool mval) {record_violation_history = mval;}
	double GetRecordViolation() {return record_violation_history;}

				/// Access the vector with the (hopefully decreasing:)
				/// history of values showing the max constraint violation 
				/// changing during iterations, up to solution.
				/// Note that you must set SetRecordViolation(true) to use it.
	std::vector<double>& GetViolationHistory() {return violation_history;};

				/// Access the vector with the (hopefully decreasing:)
				/// history of max delta in lambda multiplier values.
				/// Note that you must set SetRecordViolation(true) to use it.
	std::vector<double>& GetDeltalambdaHistory() {return dlambda_history;};


protected:
				// This method MUST be called by all iterative
				// methods INSIDE their iteration loops (at the end). If you use
				// SetRecordViolation(true), the violation history 
				// will be automatically updated by this callback.
				// Note: 'iternum' must be 0 for 1st iteration, etc.
	void AtIterationEnd(double mmaxviolation, double mdeltalambda, unsigned int iternum)
			{
				if (!record_violation_history) return;
				if (iternum!= violation_history.size())
				{
					violation_history.clear();
					violation_history.resize(iternum);
				}
				if (iternum!= dlambda_history.size())
				{
					dlambda_history.clear();
					dlambda_history.resize(iternum);
				}
				violation_history.push_back(mmaxviolation);
				dlambda_history.push_back(mdeltalambda);
			}
};



} // END_OF_NAMESPACE____




#endif  // END of ChLcpIterativeSolver.h
