// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
// Base class for a vehicle driveline.
//
// =============================================================================

#include "subsys/brake/ChBrakeSimple.h"


namespace chrono {




ChBrakeSimple::ChBrakeSimple(ChSharedPtr<ChLinkLockRevolute> mhub)
{
	modulation =1;
	maxtorque =100;

	ChSystem* my_system = mhub->GetSystem();

	// Reuse the same bodies and link coordinate of the hub revolute joint...
	ChSharedPtr<ChBodyFrame> mbf1(mhub->GetBody1());
	mhub->GetBody1()->AddRef(); // because mbf1(mhub->GetBody1()) got a plain pointer, so transformed to shared 
	ChSharedPtr<ChBodyFrame> mbf2(mhub->GetBody2());
	mhub->GetBody2()->AddRef(); // because mbf2(mhub->GetBody2()) got a plain pointer, so transformed to shared 
	ChSharedPtr<ChBody> mb1 = mbf1.DynamicCastTo<ChBody>();
	ChSharedPtr<ChBody> mb2 = mbf2.DynamicCastTo<ChBody>();

	this->mbrake = ChSharedPtr<ChLinkBrake>(new ChLinkBrake); 
	this->mbrake->Initialize( mb1,  mb2,  mhub->GetMarker2()->GetCoord());
	my_system->AddLink(this->mbrake);

}



double ChBrakeSimple::GetBrakeSpeed()
{
	return this->mbrake->GetRelWvel().Length();
}

void ChBrakeSimple::ApplyBrakeModulation(double mmodulation)
{
	this->modulation = mmodulation;

	this->mbrake->Set_brake_torque(this->modulation*this->maxtorque);
}
		
double ChBrakeSimple::GetBrakeTorque()
{
	return (this->modulation*this->maxtorque);
}

		/// Set the max braking torque (for modulation =1)
void ChBrakeSimple::SetMaxBrakingTorque(double maxt)
{
	this->maxtorque = maxt;
}


}  // end namespace chrono
