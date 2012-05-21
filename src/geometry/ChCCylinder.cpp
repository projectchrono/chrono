//////////////////////////////////////////////////
//  
//   ChCCylinder.cpp
//
// ------------------------------------------------
// 	 Copyright 2006 Alessandro Tasora 
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdio.h>


#include "ChCCylinder.h"


namespace chrono 
{
namespace geometry 
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChCylinder> a_registration_ChCylinder;


void ChCylinder::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChGeometry::StreamOUT(mstream);

		// stream out all member data
	mstream << p1;
	mstream << p2;
	mstream << rad;
}

void ChCylinder::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChGeometry::StreamIN(mstream);

		// stream in all member data
	mstream >> p1;
	mstream >> p2;
	mstream >> rad;
}


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

