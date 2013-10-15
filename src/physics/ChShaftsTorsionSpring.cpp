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
//   ChShaftsTorsionSpring.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChShaftsTorsionSpring.h"
#include "physics/ChSystem.h"
#include "physics/ChShaft.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaftsTorsionSpring> a_registration_ChShaftsTorsionSpring;



//////////////////////////////////////
//////////////////////////////////////



ChShaftsTorsionSpring::ChShaftsTorsionSpring ()
{
	this->torque_kr= 0;
	this->stiffness = 0;
	this->damping = 0;

	SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID
}


ChShaftsTorsionSpring::~ChShaftsTorsionSpring ()
{
	
}

void ChShaftsTorsionSpring::Copy(ChShaftsTorsionSpring* source)
{
		// copy the parent class data...
	ChShaftsCouple::Copy(source);

		// copy class data
	torque_kr = source->torque_kr;
	stiffness = source->stiffness;
	damping   = source->damping;
}



void ChShaftsTorsionSpring::Update (double mytime)
{
		// Inherit time changes of parent class
	ChShaftsCouple::Update(mytime);
	
		// update class data
	torque_kr = - (this->GetRelativeRotation() * this->stiffness + this->GetRelativeRotation_dt() * this->damping);
}




////////// LCP INTERFACES ////


void ChShaftsTorsionSpring::VariablesFbLoadForces(double factor)
{
	// add applied torques to 'fb' vector
	this->shaft1->Variables().Get_fb().ElementN(0) +=  this->torque_kr * factor;
	this->shaft2->Variables().Get_fb().ElementN(0) += -this->torque_kr * factor;
}



//////// FILE I/O

void ChShaftsTorsionSpring::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChShaftsCouple::StreamOUT(mstream);

		// stream out all member data
	mstream << this->stiffness;
	mstream << this->damping;
}

void ChShaftsTorsionSpring::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChShaftsCouple::StreamIN(mstream);

		// deserialize class
	mstream >> this->stiffness;
	mstream >> this->damping;
}






} // END_OF_NAMESPACE____


/////////////////////
