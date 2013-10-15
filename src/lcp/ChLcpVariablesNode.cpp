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
//   ChLcpVariablesNode.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 

#include "ChLcpVariablesNode.h"
 

namespace chrono 
{



ChLcpVariablesNode& ChLcpVariablesNode::operator=(const ChLcpVariablesNode& other)
{
	if (&other == this) return *this;

	// copy parent class data
	ChLcpVariables::operator=(other);

	// copy class data
	user_data = other.user_data;
	mass = other.mass;

	return *this;
}



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpVariablesNode> a_registration_ChLcpVariablesNode;



} // END_OF_NAMESPACE____


