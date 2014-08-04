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

#ifndef CHLCPCONSTRAINTNODEFRICTIONT_H
#define CHLCPCONSTRAINTNODEFRICTIONT_H

//////////////////////////////////////////////////
//
//   ChLcpConstraintNodeFrictionT.h
//
//  Class used to represent friction constraint
// between a 3DOF node and a 6DOF body.
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpConstraintTwoGeneric.h"
#include "ChLcpVariablesBody.h"
#include "ChLcpVariablesNode.h"

namespace chrono
{


/// Class used to represent friction constraint
/// between a 3DOF node and a 6DOF body.

class ChApi ChLcpConstraintNodeFrictionT : public ChLcpConstraintTwoGeneric
{
	CH_RTTI(ChLcpConstraintNodeFrictionT, ChLcpConstraintTwoGeneric)

			//
			// DATA
			//

protected:


public:

			//
			// CONSTRUCTORS
			//
						/// Default constructor
	ChLcpConstraintNodeFrictionT()
					{
						mode = CONSTRAINT_FRIC; 
					};

						/// Construct and immediately set references to variables,
						/// also setting the  and the normal constraint
						/// other tangential constraint (the latter is mandatory only
						/// for the second of the two tangential constraints)
	ChLcpConstraintNodeFrictionT( ChLcpVariablesBody* mvariables_a,
								ChLcpVariablesNode* mvariables_b)
				: ChLcpConstraintTwoGeneric(mvariables_a, mvariables_b)
					{
						mode = CONSTRAINT_FRIC; 
					};

						/// Copy constructor
	ChLcpConstraintNodeFrictionT(const ChLcpConstraintNodeFrictionT& other) 
				: ChLcpConstraintTwoGeneric(other)
					{
					}

	virtual ~ChLcpConstraintNodeFrictionT() {};

	virtual ChLcpConstraint* new_Duplicate () {return new ChLcpConstraintNodeFrictionT(*this);};

					/// Assignment operator: copy from other object
	ChLcpConstraintNodeFrictionT& operator=(const ChLcpConstraintNodeFrictionT& other)
					{
						if (&other == this)
							return *this;

						// copy parent class data
						ChLcpConstraintTwoGeneric::operator=(other);

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




#endif  
