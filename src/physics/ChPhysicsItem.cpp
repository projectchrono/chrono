//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010, 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChPhysicsItem.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChPhysicsItem.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{





// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegisterABSTRACT<ChPhysicsItem> a_registration_ChPhysicsItem;



void ChPhysicsItem::Copy(ChPhysicsItem* source)
{
    // first copy the parent class data...
    ChObj::Copy(source);

	// copy other class data
	system=0; // do not copy - must be initialized with insertion in system.
	
	this->offset_x = source->offset_x;
	this->offset_w = source->offset_w;
	this->offset_L = source->offset_L;

	this->assets = source->assets;  // copy the list of shared pointers to assets
}




void ChPhysicsItem::GetTotalAABB(ChVector<>& bbmin, ChVector<>& bbmax)
{
	bbmin.Set(-1e200, -1e200, -1e200);
	bbmax.Set( 1e200,  1e200,  1e200);
}
				
void ChPhysicsItem::GetCenter(ChVector<>& mcenter)
{
	ChVector<> mmin, mmax;
	this->GetTotalAABB(mmin, mmax);
	mcenter = (mmin+mmax)*0.5;
}




/////////
///////// FILE I/O
/////////



void ChPhysicsItem::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChObj::StreamOUT(mstream);

		// stream out all member data
	
}

void ChPhysicsItem::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChObj::StreamIN(mstream);

		// stream in all member data

}











} // END_OF_NAMESPACE____


