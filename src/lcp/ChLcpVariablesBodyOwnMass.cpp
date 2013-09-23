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
//   ChLcpVariablesBodyOwnMass.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 

#include "ChLcpVariablesBodyOwnMass.h"
 

namespace chrono 
{



ChLcpVariablesBodyOwnMass& ChLcpVariablesBodyOwnMass::operator=(const ChLcpVariablesBodyOwnMass& other)
{
	if (&other == this) return *this;

	// copy parent class data
	ChLcpVariablesBody::operator=(other);

	// copy class data
	mass= other.mass;
	inv_mass = other.inv_mass;

	inertia = other.inertia;
	inv_inertia = other.inv_inertia;

	return *this;
}



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpVariablesBodyOwnMass> a_registration_ChLcpVariablesBodyOwnMass;



} // END_OF_NAMESPACE____


