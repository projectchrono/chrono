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



#include "physics/ChLinkBase.h"
#include "physics/ChGlobal.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{






// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegisterABSTRACT<ChLinkBase> a_registration_ChLinkBase;


                // BUILDERS
ChLinkBase::ChLinkBase ()
{
	broken = false;
	valid = true;
    disabled = false;
 
    SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID
}



            // DESTROYER
ChLinkBase::~ChLinkBase ()
{

}


void ChLinkBase::Copy(ChLinkBase* source)
{
    // first copy the parent class data...
    ChPhysicsItem::Copy(source);

	broken = source->broken;
	valid = source->valid;
    disabled = source->disabled;
}




// Define some  link-specific flags for backward compatibility

#define LF_INACTIVE		(1L << 0)
#define LF_BROKEN		(1L << 2)
#define LF_DISABLED	    (1L << 4)



void ChLinkBase::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(12);

		// serialize parent class too
	ChPhysicsItem::StreamOUT(mstream);

		// stream out all member data
	mstream << disabled;
	mstream << valid;
	mstream << broken;
}

void ChLinkBase::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
	
		// deserialize parent class too
	ChPhysicsItem::StreamIN(mstream);

	mstream >> disabled;
	mstream >> valid;
	mstream >> broken;
	
}









} // END_OF_NAMESPACE____


