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
//   ChLcpVariablesBodySharedMass.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 

#include "ChLcpVariablesBodySharedMass.h"
 

namespace chrono 
{



ChLcpVariablesBodySharedMass& ChLcpVariablesBodySharedMass::operator=(const ChLcpVariablesBodySharedMass& other)
{
	if (&other == this) return *this;

	// copy parent class data
	ChLcpVariablesBody::operator=(other);

	// copy class data
	sharedmass = other.sharedmass;

	return *this;
}



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpVariablesBodySharedMass> a_registration_ChLcpVariablesBodySharedMass;



} // END_OF_NAMESPACE____


