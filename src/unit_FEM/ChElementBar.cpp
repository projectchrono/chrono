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
// File author: Alessandro Tasora


#include "ChElementBar.h"


namespace chrono
{
namespace fem
{



ChElementBar::ChElementBar()
{
	nodes.resize(2);

	area = 0.01*0.01; // default area: 1cmx1cm 
	density = 1000;   // default density: water
	E = 0.01e9;		  // default stiffness: rubber
	rdamping = 0.01;  // default raleygh damping.

	length = 0;		// will be computed by Setup(), later
	mass = 0;		// will be computed by Setup(), later
}


ChElementBar::~ChElementBar()
{
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____








