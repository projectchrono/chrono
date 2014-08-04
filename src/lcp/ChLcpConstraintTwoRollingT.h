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

#ifndef CHLCPCONSTRAINTTWOROLLINGT_H
#define CHLCPCONSTRAINTTWOROLLINGT_H

//////////////////////////////////////////////////
//
//   ChLcpConstraintTwoRollingT.h
//
//  Class used to represent rolling friction constraint
// between two ChLcpVariable() items.
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpConstraintTwoBodies.h"
#include "ChLcpVariables.h"

namespace chrono
{


/// This is used to make the rolling friction constraint. This 
/// must be used in twice copy each ChLcpConstraintTwoRollingN

class ChApi ChLcpConstraintTwoRollingT: public ChLcpConstraintTwoBodies
{
	CH_RTTI(ChLcpConstraintTwoRollingT, ChLcpConstraintTwoBodies)

			//
			// DATA
			//

protected:


public:

			//
			// CONSTRUCTORS
			//
						/// Default constructor
	ChLcpConstraintTwoRollingT()
					{
						mode = CONSTRAINT_FRIC; 
					};

						/// Construct and immediately set references to variables,
						/// also setting the  and the normal constraint
						/// other tangential constraint (the latter is mandatory only
						/// for the second of the two tangential constraints)
	ChLcpConstraintTwoRollingT( ChLcpVariablesBody* mvariables_a,
								ChLcpVariablesBody* mvariables_b)
				: ChLcpConstraintTwoBodies(mvariables_a, mvariables_b)
					{
						mode = CONSTRAINT_FRIC; 
					};

						/// Copy constructor
	ChLcpConstraintTwoRollingT(const ChLcpConstraintTwoRollingT& other) 
				: ChLcpConstraintTwoBodies(other)
					{
					}

	virtual ~ChLcpConstraintTwoRollingT() {};

	virtual ChLcpConstraint* new_Duplicate () {return new ChLcpConstraintTwoRollingT(*this);};

					/// Assignment operator: copy from other object
	ChLcpConstraintTwoRollingT& operator=(const ChLcpConstraintTwoRollingT& other)
					{
						if (&other == this)
							return *this;

						// copy parent class data
						ChLcpConstraintTwoBodies::operator=(other);

						return *this;
					}


			//
			// FUNCTIONS
			//

					/// Tells that this constraint is not linear, that is: it cannot
					/// be solved with a plain simplex solver.
	virtual bool IsLinear() const { return false; }

					/// The constraint is satisfied?
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




#endif  // END of ChLcpConstraintTwoRollingT.h
