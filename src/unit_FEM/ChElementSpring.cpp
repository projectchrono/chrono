//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChElementSpring.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChElementSpring.h"


namespace chrono
{
namespace fem
{



ChElementSpring::ChElementSpring()
{
	spring_k = 1.0; 
	damper_r = 0.01; 

	nodes.resize(2);
}


ChElementSpring::~ChElementSpring()
{
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____








