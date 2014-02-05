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




#include "physics/ChLink.h"
#include "physics/ChGlobal.h"
#include "physics/ChSystem.h"
#include "physics/ChExternalObject.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{






// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLink> a_registration_ChLink;


                // BUILDERS
ChLink::ChLink ()
{
    Body1 = NULL;
    Body2 = NULL;

    react_force = VNULL;
    react_torque = VNULL;
 
    SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID
}



            // DESTROYER
ChLink::~ChLink ()
{

}


void ChLink::Copy(ChLink* source)
{
    // first copy the parent class data...
    ChLinkBase::Copy(source);

    Body1 = 0;
    Body2 = 0;
    system = 0;

	react_force = source->react_force;
	react_torque = source->react_torque;
}


ChLink* ChLink::new_Duplicate ()   // inherited classes:  Link* MyInheritedLink::new_Duplicate()
{
    ChLink* m_l;
    m_l = new ChLink;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}



 


void ChLink::Set2Dmode(int mode)
{
    // Nothing specific to do on mask, 'cause base link.
    // Chld classes can implement this method,
}






////////////////////////////////////
///
///    UPDATING PROCEDURES



/////////   1-   UPDATE TIME
/////////

void ChLink::UpdateTime (double time)
{
    ChTime = time;
}






/////////
/////////   COMPLETE UPDATE
/////////
/////////

void ChLink::Update (double time)
{
    // 1 -
    UpdateTime(time);

}


void ChLink::Update ()
{
    Update(ChTime); // use the same time
}






void ChLink::UpdateExternalGeometry ()
{
	if (GetExternalObject())
		GetExternalObject()->onChronoChanged();
}




/////////
///////// FILE I/O
/////////


// Define some  link-specific flags for backward compatibility

#define LF_INACTIVE		(1L << 0)
#define LF_BROKEN		(1L << 2)
#define LF_DISABLED	    (1L << 4)



void ChLink::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(11);

		// serialize parent class too
	ChLinkBase::StreamOUT(mstream);

		// stream out all member data
	//...
}

void ChLink::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
	
		// deserialize parent class too
	if (version <11)
	{	
		ChObj::StreamIN(mstream);
	}
	if (version == 11)
	{
		ChPhysicsItem::StreamIN(mstream);
	}
	if (version >= 12)
	{
		ChLinkBase::StreamIN(mstream);
	}

		// deserialize class data
	if(version ==1)
	{
		int mylflag; // was 'long' in v.1 but 'long' streaming support is removed
		mstream >> mylflag;
		valid = !(mylflag & LF_INACTIVE);
		disabled = (mylflag & LF_DISABLED)!=0;
		broken = (mylflag & LF_BROKEN)!=0;
	}
	if((version >=2) && (version <12))
	{
		mstream >> disabled;
		mstream >> valid;
		mstream >> broken;
	}

	//...
	
}











} // END_OF_NAMESPACE____


