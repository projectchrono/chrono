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
//   ChLcpConstraintTwoFrictionT.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLcpConstraintTwoFrictionT.h"


namespace chrono
{

// Register into the object factory, to enable run-time 
// dynamic creation and persistence
ChClassRegister<ChLcpConstraintTwoFrictionT> a_registration_ChLcpConstraintTwoFrictionT;


double ChLcpConstraintTwoFrictionT::Violation(double mc_i)
{
	return 0.0; //***TO DO*** compute true violation when in sticking?
}



void ChLcpConstraintTwoFrictionT::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChLcpConstraintTwoBodies::StreamOUT(mstream);

}
 
void ChLcpConstraintTwoFrictionT::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLcpConstraintTwoBodies::StreamIN(mstream);

}




} // END_OF_NAMESPACE____


