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
//   ChLcpConstraintTwoGenericBoxed.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver 
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

 
#include "ChLcpConstraintTwoGenericBoxed.h" 

#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpConstraintTwoGenericBoxed> a_registration_ChLcpConstraintTwoGenericBoxed;


double ChLcpConstraintTwoGenericBoxed::Violation(double mc_i) 
{
	if ((l_i- 10e-5 < l_min)||(l_i +10e-5 > l_max))
		return 0;
	return mc_i;
}


void ChLcpConstraintTwoGenericBoxed::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChLcpConstraintTwoGeneric::StreamOUT(mstream);

		// stream out all member data
	mstream << l_min;
	mstream << l_max;
} 
 


void ChLcpConstraintTwoGenericBoxed::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLcpConstraintTwoGeneric::StreamIN(mstream);

		// stream in all member data
	mstream >> l_min;
	mstream >> l_max;
}




} // END_OF_NAMESPACE____


