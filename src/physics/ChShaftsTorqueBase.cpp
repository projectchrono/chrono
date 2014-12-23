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


#include "physics/ChShaftsTorqueBase.h"
#include "physics/ChSystem.h"
#include "physics/ChShaft.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegisterABSTRACT<ChShaftsTorqueBase> a_registration_ChShaftsTorqueBase;



//////////////////////////////////////
//////////////////////////////////////



ChShaftsTorqueBase::ChShaftsTorqueBase ()
{
	this->torque= 0;

	SetIdentifier(GetUniqueIntID()); // mark with unique ID
}


ChShaftsTorqueBase::~ChShaftsTorqueBase ()
{
	
}

void ChShaftsTorqueBase::Copy(ChShaftsTorqueBase* source)
{
		// copy the parent class data...
	ChShaftsCouple::Copy(source);

		// copy class data
	torque = source->torque;
}



void ChShaftsTorqueBase::Update (double mytime)
{
		// Inherit time changes of parent class
	ChShaftsCouple::Update(mytime);
	
		// update class data
	this->torque = ComputeTorque();
}



//// STATE BOOKKEEPING FUNCTIONS

void ChShaftsTorqueBase::IntLoadResidual_F(
					const unsigned int off,		 ///< offset in R residual
					ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*F 
					const double c				 ///< a scaling factor
					)
{
	R(shaft1->Variables().GetOffset()) +=  this->torque * c;
	R(shaft2->Variables().GetOffset()) += -this->torque * c;
}


////////// LCP INTERFACES ////


void ChShaftsTorqueBase::VariablesFbLoadForces(double factor)
{
	// add applied torques to 'fb' vector
	this->shaft1->Variables().Get_fb().ElementN(0) +=  this->torque * factor;
	this->shaft2->Variables().Get_fb().ElementN(0) += -this->torque * factor;
}



//////// FILE I/O

void ChShaftsTorqueBase::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChShaftsCouple::StreamOUT(mstream);

		// stream out all member data
	mstream << this->torque;
}

void ChShaftsTorqueBase::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChShaftsCouple::StreamIN(mstream);

		// deserialize class
	mstream >> this->torque;
}






} // END_OF_NAMESPACE____


/////////////////////
