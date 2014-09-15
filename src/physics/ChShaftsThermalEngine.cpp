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



#include "physics/ChShaftsThermalEngine.h"
#include "physics/ChSystem.h"
#include "physics/ChShaft.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaftsThermalEngine> a_registration_ChShaftsThermalEngine;



//////////////////////////////////////
//////////////////////////////////////



ChShaftsThermalEngine::ChShaftsThermalEngine ()
{
	this->throttle = 1;
	this->error_backward = false;

	// default torque curve= constant zero. User will provide better fx.
	this->Tw = ChSharedPtr<ChFunction> (new ChFunction_Const(0));

	SetIdentifier(GetUniqueIntID()); // mark with unique ID
}


ChShaftsThermalEngine::~ChShaftsThermalEngine ()
{
	
}

void ChShaftsThermalEngine::Copy(ChShaftsThermalEngine* source)
{
		// copy the parent class data...
	ChShaftsTorqueBase::Copy(source);

		// copy class data
	throttle = source->throttle;
	error_backward = source->error_backward;
	this->Tw = ChSharedPtr<ChFunction>(source->Tw->new_Duplicate()); // deep copy
}



double ChShaftsThermalEngine::ComputeTorque()
{
		// COMPUTE THE TORQUE HERE!
	double mw = this->GetRelativeRotation_dt();

	if (mw <0) 
		this->error_backward = true;
	else
		this->error_backward = false;

	// get the actual torque from torque curve
	double mT = Tw->Get_y( mw );		

	// modulate it with throttle
	double modulated_T = mT * this->throttle; 

	return modulated_T;
}



//////// FILE I/O

void ChShaftsThermalEngine::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChShaftsTorqueBase::StreamOUT(mstream);

		// stream out all member data
	mstream << this->throttle;
	mstream << this->error_backward;
	mstream.AbstractWrite(this->Tw.get_ptr()); //***TODO*** proper serialize for ChSharedPtr
}

void ChShaftsThermalEngine::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChShaftsTorqueBase::StreamIN(mstream);

		// deserialize class data
	mstream >> this->throttle;
	mstream >> this->error_backward;

	ChFunction* newfun;
	mstream.AbstractReadCreate(&newfun); //***TODO*** proper deserialize for ChSharedPtr
	this->Tw = ChSharedPtr<ChFunction>(newfun);
}






} // END_OF_NAMESPACE____


/////////////////////
