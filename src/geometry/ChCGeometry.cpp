//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2006, 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//////////////////////////////////////////////////
//  
//   ChCGeometry.cpp
//
// ------------------------------------------------
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdio.h>


#include "ChCGeometry.h"


namespace chrono 
{
namespace geometry 
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChGeometry> a_registration_ChGeometry;



ChGeometry::ChGeometry() {};

ChGeometry::~ChGeometry() {};



void ChGeometry::Derive(Vector& dir, 
						const double parU, 
						const double parV, 
						const double parW)
{
	double bdf = 10e-9;
	double uA=0,uB=0;

	if (parU > 0.5)
	{
		double uB = parU;
		double uA = parU-bdf;
	}else
	{
		double uB = parU+bdf;
		double uA = parU;
	}
	Vector vA, vB;
	Evaluate(vA,uA);
	Evaluate(vB,uB);
	
	dir = (vB-vA)*(1/bdf);
}

void ChGeometry::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
}

void ChGeometry::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

