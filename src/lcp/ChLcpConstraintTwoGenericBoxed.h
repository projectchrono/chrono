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

#ifndef CHLCPCONSTRAINTTWOGENERICBOXED_H
#define CHLCPCONSTRAINTTWOGENERICBOXED_H

//////////////////////////////////////////////////
//
//   ChLcpConstraintTwoGenericBoxed.h
//
//    An 'easy' derived class for representing a
//   constraint between two ChLcpVariable items, where
//   the multiplier must be   l_min < l < l_max
//   Used with for building sparse variational problems 
//   (VI/CCP/LCP/linear problems) described by 
//   a ChLcpSystemDescriptor
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpConstraintTwoGeneric.h"

#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{


///  This class is inherited by the base ChLcpConstraintTwoGeneric(),
/// which can be used for most pairwise constraints, and adds the
/// feature that the multiplier must be   
///        l_min < l < l_max
/// that is, the multiplier is 'boxed'. 
/// Note that, if l_min = 0 and l_max = infinite, this can work
/// also as an unilateral constraint..
///  Before starting the LCP solver one must provide the proper
/// values in constraints (and update them if necessary), i.e.
/// must set at least the c_i and b_i values, and jacobians.

class ChApi ChLcpConstraintTwoGenericBoxed : public ChLcpConstraintTwoGeneric
{
	CH_RTTI(ChLcpConstraintTwoGenericBoxed, ChLcpConstraintTwoGeneric)

			//
			// DATA
			//

protected:
	double l_min;
	double l_max;

public:

			//
			// CONSTRUCTORS
			//
						/// Default constructor
	ChLcpConstraintTwoGenericBoxed()
					{
						l_min = -1.; 
						l_max =  1.;
					};

						/// Construct and immediately set references to variables
	ChLcpConstraintTwoGenericBoxed(ChLcpVariables* mvariables_a, ChLcpVariables* mvariables_b) 
			: ChLcpConstraintTwoGeneric(mvariables_a, mvariables_b)
					{
						l_min = -1.; 
						l_max =  1.;
					};

						/// Copy constructor
	ChLcpConstraintTwoGenericBoxed(const ChLcpConstraintTwoGenericBoxed& other) 
			: ChLcpConstraintTwoGeneric(other)
					{
						l_min = other.l_min; 
						l_max = other.l_max;
					}

	virtual ~ChLcpConstraintTwoGenericBoxed()
					{
					};


	virtual ChLcpConstraintTwoGenericBoxed* new_Duplicate () {return new ChLcpConstraintTwoGenericBoxed(*this);};

					/// Assignment operator: copy from other object
	ChLcpConstraintTwoGenericBoxed& operator=(const ChLcpConstraintTwoGenericBoxed& other)
					{
						if (&other == this) return *this;

						// copy parent class data
						ChLcpConstraintTwoGeneric::operator=(other);

						l_min = other.l_min; 
						l_max = other.l_max;

						return *this;
					}


			//
			// FUNCTIONS
			//
					 /// Set lower/upper limit for the multiplier.
	void SetBoxedMinMax(double mmin, double mmax) 
					{
						assert (mmin <= mmax);
						l_min = mmin;
						l_max = mmax;
					}

					/// Get the lower limit for the multiplier
	double GetBoxedMin() {return l_min;}
					/// Get the upper limit for the multiplier
	double GetBoxedMax() {return l_max;}

				/// For iterative solvers: project the value of a possible
				/// 'l_i' value of constraint reaction onto admissible orthant/set.
				/// This 'boxed implementation overrides the default do-nothing case.
	virtual void Project()
					{
						if (l_i < l_min)
							l_i=l_min;
						if (l_i > l_max)
							l_i=l_max;
					}

				/// Given the residual of the constraint computed as the
				/// linear map  mc_i =  [Cq]*q + b_i + cfm*l_i , returns the
				/// violation of the constraint, considering inequalities, etc.
	virtual double Violation(double mc_i);

			//
			// STREAMING
			//

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream);
};




} // END_OF_NAMESPACE____



#include "core/ChMemorynomgr.h" // back to default new/delete/malloc/calloc etc. Avoid conflicts with system libs.


#endif  
