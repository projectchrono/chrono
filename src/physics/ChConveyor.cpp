//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChConveyor.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

    
#include <stdlib.h>
#include <algorithm>

#include "core/ChTransform.h"
#include "physics/ChConveyor.h"
#include "physics/ChSystem.h"

#include "collision/ChCModelBulletBody.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{

using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChConveyor> a_registration_ChConveyor;



//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR CONVEYOR


ChConveyor::ChConveyor (double xlength, double ythick, double zwidth)
{
	conveyor_speed = 1.0; 
	
	conveyor_plate = new ChBody;

	conveyor_plate->SetMaterialSurface(this->GetMaterialSurface());

	conveyor_plate->GetCollisionModel()->ClearModel();
	conveyor_plate->GetCollisionModel()->AddBox(xlength*0.5, ythick*0.5, zwidth*0.5); 
	conveyor_plate->GetCollisionModel()->BuildModel();
	conveyor_plate->SetCollide(true);


	internal_link  = new ChLinkLockLock;
	internal_link->SetMotion_X(new ChFunction_Ramp);

	ChSharedPtr<ChMarker> mmark1(new ChMarker);
	ChSharedPtr<ChMarker> mmark2(new ChMarker);
	this->AddMarker(mmark1);
	this->conveyor_plate->AddMarker(mmark2);

	internal_link->ReferenceMarkers(mmark1.get_ptr(), mmark2.get_ptr());
}


ChConveyor::~ChConveyor ()
{
	if (internal_link) delete internal_link;
	if (conveyor_plate) delete conveyor_plate;
}

void ChConveyor::Copy(ChConveyor* source)
{
		// copy the parent class data...
	ChBody::Copy(source);

	this->conveyor_speed	= source->conveyor_speed;

	this->internal_link = source->internal_link;
	this->conveyor_plate = source->conveyor_plate;
}






//// LCP INTERFACE

void ChConveyor::InjectVariables(ChLcpSystemDescriptor& mdescriptor)
{	
			// inherit parent class
	ChBody::InjectVariables(mdescriptor);

	this->conveyor_plate->InjectVariables(mdescriptor);
}


void ChConveyor::VariablesFbReset()
{
				// inherit parent class
	ChBody::VariablesFbReset();

	this->conveyor_plate->VariablesFbReset();
}


void ChConveyor::VariablesFbLoadForces(double factor)
{
					// inherit parent class
	ChBody::VariablesFbLoadForces(factor);

	this->conveyor_plate->VariablesFbLoadForces(factor);
}

void ChConveyor::VariablesFbIncrementMq()
{
					// inherit parent class
	ChBody::VariablesFbIncrementMq();

	this->conveyor_plate->VariablesFbIncrementMq();
}

void ChConveyor::VariablesQbLoadSpeed()
{
					// inherit parent class
	ChBody::VariablesQbLoadSpeed();

	this->conveyor_plate->VariablesQbLoadSpeed();
}


void ChConveyor::VariablesQbSetSpeed(double step)
{
						// inherit parent class
	ChBody::VariablesQbSetSpeed(step);

	this->conveyor_plate->VariablesQbSetSpeed(step);
}


void ChConveyor::VariablesQbIncrementPosition(double dt_step)
{
					// inherit parent class
	ChBody::VariablesQbIncrementPosition(dt_step);

	this->conveyor_plate->VariablesQbIncrementPosition(dt_step);
}


void ChConveyor::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	this->internal_link->InjectConstraints(mdescriptor);
}

void ChConveyor::ConstraintsBiReset()
{
	this->internal_link->ConstraintsBiReset();
}
 
void ChConveyor::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	this->internal_link->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
}

void ChConveyor::ConstraintsBiLoad_Ct(double factor)
{
	this->internal_link->ConstraintsBiLoad_Ct(factor);
}

void ChConveyor::ConstraintsBiLoad_Qc(double factor)
{
	this->internal_link->ConstraintsBiLoad_Qc(factor);
}

void ChConveyor::ConstraintsLoadJacobians()
{
	this->internal_link->ConstraintsLoadJacobians();
}
 
void ChConveyor::ConstraintsFetch_react(double factor)
{
	this->internal_link->ConstraintsFetch_react(factor);
}

void  ChConveyor::ConstraintsLiLoadSuggestedSpeedSolution()
{
	this->internal_link->ConstraintsLiLoadSuggestedSpeedSolution();
}

void  ChConveyor::ConstraintsLiLoadSuggestedPositionSolution()
{
	this->internal_link->ConstraintsLiLoadSuggestedPositionSolution();
}

void  ChConveyor::ConstraintsLiFetchSuggestedSpeedSolution()
{
	this->internal_link->ConstraintsLiFetchSuggestedSpeedSolution();
}

void  ChConveyor::ConstraintsLiFetchSuggestedPositionSolution()
{
	this->internal_link->ConstraintsLiFetchSuggestedPositionSolution();
}






					
void ChConveyor::Update (double mytime)
{
				// inherit parent class function
	ChBody::Update(mytime);
	
	if (this->GetBodyFixed())
	{
		double largemass = 100000;
		this->conveyor_plate->SetMass(largemass);
		this->conveyor_plate->SetInertiaXX(ChVector<>(largemass,largemass,largemass));
		this->conveyor_plate->SetInertiaXY(ChVector<>(0,0,0));
	}
	else
	{
		this->conveyor_plate->SetMass(this->GetMass());
		this->conveyor_plate->SetInertiaXX(this->GetInertiaXX());
		this->conveyor_plate->SetInertiaXY(this->GetInertiaXY());
	}

	this->conveyor_plate->SetSystem(this->GetSystem());

	this->conveyor_plate->SetMaterialSurface(this->GetMaterialSurface());

				// keep the plate always at the same position of the main reference
	this->conveyor_plate->SetCoord(this->GetCoord());
	this->conveyor_plate->SetCoord_dt(this->GetCoord_dt());
				// keep the plate always at the same speed of the main reference, plus the conveyor speed on X local axis
	this->conveyor_plate->SetPos_dt(this->GetPos_dt() + (ChVector<>(conveyor_speed,0,0) >> (*this) ) );

	conveyor_plate->Update(mytime);

	this->internal_link->SetSystem(this->GetSystem());

	((ChFunction_Ramp*)internal_link->GetMotion_X())->Set_ang(-conveyor_speed); 
	((ChFunction_Ramp*)internal_link->GetMotion_X())->Set_y0(+conveyor_speed*this->GetChTime()); // always zero pos. offset (trick)

	this->internal_link->Update(mytime);
}





void ChConveyor::SyncCollisionModels()
{
						// inherit parent class
	ChBody::SyncCollisionModels();

	this->conveyor_plate->SetSystem(this->GetSystem());

	this->conveyor_plate->SyncCollisionModels();
}

void ChConveyor::AddCollisionModelsToSystem() 
{
						// inherit parent class
	ChBody::AddCollisionModelsToSystem();

	this->conveyor_plate->SetSystem(this->GetSystem());

	this->conveyor_plate->AddCollisionModelsToSystem();
}

void ChConveyor::RemoveCollisionModelsFromSystem() 
{
						// inherit parent class
	ChBody::RemoveCollisionModelsFromSystem();

	this->conveyor_plate->SetSystem(this->GetSystem());

	this->conveyor_plate->RemoveCollisionModelsFromSystem();
}



//////// FILE I/O

void ChConveyor::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChBody::StreamOUT(mstream);

		// stream out all member data
	mstream << this->conveyor_speed;
	this->conveyor_plate->StreamOUT(mstream);
	this->internal_link->StreamOUT(mstream);
}

void ChConveyor::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChBody::StreamIN(mstream);

		// stream in all member data
	mstream >> this->conveyor_speed;
	this->conveyor_plate->StreamIN(mstream);
	this->internal_link->StreamIN(mstream);
}





} // END_OF_NAMESPACE____


/////////////////////
