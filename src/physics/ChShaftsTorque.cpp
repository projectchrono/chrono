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



#include "physics/ChShaftsTorque.h"
#include "physics/ChSystem.h"
#include "physics/ChShaft.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaftsTorque> a_registration_ChShaftsTorque;



//////////////////////////////////////
//////////////////////////////////////



ChShaftsTorque::ChShaftsTorque ()
{


	SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID
}


ChShaftsTorque::~ChShaftsTorque ()
{
	
}

void ChShaftsTorque::Copy(ChShaftsTorque* source)
{
		// copy the parent class data...
	ChShaftsTorqueBase::Copy(source);

}



double ChShaftsTorque::ComputeTorque()
{
		// COMPUTE THE TORQUE HERE!
	// Nothing, just leave the user-set   this->torque 
	return this->torque;
}




//////// FILE I/O

void ChShaftsTorque::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChShaftsTorqueBase::StreamOUT(mstream);

		// stream out all member data

}

void ChShaftsTorque::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChShaftsTorqueBase::StreamIN(mstream);

		// deserialize class

}






} // END_OF_NAMESPACE____


/////////////////////
