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

///////////////////////////////////////////////////
//
//   ChLcpConstraintThree.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver 
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

  
#include "ChLcpConstraintThree.h"

#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegisterABSTRACT<ChLcpConstraintThree> a_registration_ChLcpConstraintThree;



ChLcpConstraintThree& ChLcpConstraintThree::operator=(const ChLcpConstraintThree& other)
{
	if (&other == this) return *this;

	// copy parent class data
	ChLcpConstraint::operator=(other);

	this->variables_a = other.variables_a;
	this->variables_b = other.variables_b;
	this->variables_c = other.variables_c;

	return *this;
}


void ChLcpConstraintThree::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChLcpConstraint::StreamOUT(mstream);

		// stream out all member data
	// NOTHING INTERESTING TO SERIALIZE (pointers to variables must be rebound in run-time.)

}

void ChLcpConstraintThree::StreamIN(ChStreamInBinary& mstream) 
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLcpConstraint::StreamIN(mstream);

		// stream in all member data
	// NOTHING INTERESTING TO DESERIALIZE (pointers to variables must be rebound in run-time.)

}




} // END_OF_NAMESPACE____


