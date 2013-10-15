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

#ifndef CHLCPCONSTRAINTTWOCONTACT_H
#define CHLCPCONSTRAINTTWOCONTACT_H

//////////////////////////////////////////////////
//
//   ChLcpConstraintTwoContact.h
//
//    An 'easy' derived class for representing a
//   constraint between two ChLcpVariable items.
//   Used with for building sparse variational problems 
//   (VI/CCP/LCP/linear problems) described by 
//   a ChLcpSystemDescriptor
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpConstraintTwoBodies.h"

#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{


///  This class is inherited by the ChLcpConstraintTwoBodies(),
/// It is used to represent the normal reaction between two objects
/// ONLY when also two ChLcpConstraintTwoFriction objects are
/// used to represent friction. (If these two tangent constraint
/// are not used, for frictionless case, please use a simple ChConstraintTwo
/// with the CONSTRAINT_UNILATERAL mode.) 
/// Differently from an unilateral constraint, this does not enforce
/// projection on positive constraint, since it will be up to the 'companion'
/// ChLcpConstraintTwoFriction objects to call a projection on the cone, by
/// modifying all the three components (normal, u, v) at once.

class ChLcpConstraintTwoContact : public ChLcpConstraintTwoBodies
{
	CH_RTTI(ChLcpConstraintTwoContact, ChLcpConstraintTwoBodies)

public:

			//
			// CONSTRUCTORS
			//
						/// Default constructor
	ChLcpConstraintTwoContact() 
					{
						mode = CONSTRAINT_FRIC;
					};

						/// Construct and immediately set references to variables
	ChLcpConstraintTwoContact(ChLcpVariablesBody* mvariables_a, ChLcpVariablesBody* mvariables_b)
			: ChLcpConstraintTwoBodies(mvariables_a, mvariables_b)
					{
						mode = CONSTRAINT_FRIC;
					};

						/// Copy constructor
	ChLcpConstraintTwoContact(const ChLcpConstraintTwoContact& other) 
			: ChLcpConstraintTwoBodies(other)
					{
					}

	virtual ~ChLcpConstraintTwoContact()
					{
					};

	virtual ChLcpConstraintTwoContact* new_Duplicate () {return new ChLcpConstraintTwoContact(*this);};

					/// Assignment operator: copy from other object
	ChLcpConstraintTwoContact& operator=(const ChLcpConstraintTwoContact& other)
					{
						if (&other == this)
							return *this;
						// copy parent class data
						ChLcpConstraintTwoBodies::operator=(other);
						// no data to copy
						return *this;
					}


			//
			// FUNCTIONS
			//



	virtual void Project()
					{
						// This is here to prevent this constraint to do Project(), during
						// the LCP iterative solver, since a special projection (involving three 
						// components, i.e. this normal reaction and the two tangent u v friction
						// reactions) will be made by one of the two companions ChLcpConstraintTwoFriction 
						// objects - only when needed.
					}


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
