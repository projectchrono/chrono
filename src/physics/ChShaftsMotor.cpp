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
//   ChShaftsMotor.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChShaftsMotor.h"
#include "physics/ChSystem.h"
#include "physics/ChShaft.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaftsMotor> a_registration_ChShaftsMotor;



//////////////////////////////////////
//////////////////////////////////////



ChShaftsMotor::ChShaftsMotor ()
{
	this->motor_torque= 0;
	this->motor_mode = MOT_MODE_TORQUE;
	this->motor_set_rot = 0;
	this->motor_set_rot_dt = 0;

	this->cache_li_speed =0.f;
	this->cache_li_pos = 0.f;

	SetIdentifier(ChGLOBALS().GetUniqueIntID()); // mark with unique ID
}


ChShaftsMotor::~ChShaftsMotor ()
{
	
}

void ChShaftsMotor::Copy(ChShaftsMotor* source)
{
		// copy the parent class data...
	ChShaftsCouple::Copy(source);

		// copy class data
	motor_torque = source->motor_torque;
	motor_mode = source->motor_mode;
	motor_set_rot = source->motor_set_rot;
	motor_set_rot_dt = source->motor_set_rot_dt;

	cache_li_speed = source->cache_li_speed;
	cache_li_pos = source->cache_li_pos;
}


int ChShaftsMotor::Initialize(ChSharedPtr<ChShaft>& mshaft1, ChSharedPtr<ChShaft>& mshaft2)
{
	// Parent class initialize
	if (!ChShaftsCouple::Initialize(mshaft1, mshaft2)) return false;

	ChShaft* mm1 = mshaft1.get_ptr();
	ChShaft* mm2 = mshaft2.get_ptr();

	this->constraint.SetVariables(&mm1->Variables(), &mm2->Variables());

	this->SetSystem(this->shaft1->GetSystem());
	return true;
}


void ChShaftsMotor::Update (double mytime)
{
		// Inherit time changes of parent class
	ChShaftsCouple::Update(mytime);
	
		// update class data
	// ...
}




////////// LCP INTERFACES ////


void ChShaftsMotor::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	//if (!this->IsActive())
	//	return;
	if (motor_mode != MOT_MODE_TORQUE)
		mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsMotor::ConstraintsBiReset()
{
	if (motor_mode != MOT_MODE_TORQUE)
		constraint.Set_b_i(0.);
}
 
void ChShaftsMotor::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	//if (!this->IsActive())
	//	return;

	if (motor_mode != MOT_MODE_TORQUE)
	{
		double res;

		if (motor_mode == MOT_MODE_SPEED)
			res = 0; // no need to stabilize positions

		if (motor_mode == MOT_MODE_ROTATION)
			res = this->GetMotorRot() - this->motor_set_rot; 

		constraint.Set_b_i(constraint.Get_b_i() +  factor * res);
	}
}

void ChShaftsMotor::ConstraintsBiLoad_Ct(double factor)
{
	//if (!this->IsActive())
	//	return;

	if (motor_mode == MOT_MODE_SPEED)
	{
		double ct = this->motor_set_rot_dt;
		constraint.Set_b_i(constraint.Get_b_i() +  factor * ct);
	}
}

void ChShaftsMotor::ConstraintsFbLoadForces(double factor)
{
	if (motor_mode == MOT_MODE_TORQUE)
	{
		shaft1->Variables().Get_fb().ElementN(0) +=  motor_torque*factor;
		shaft2->Variables().Get_fb().ElementN(0) += -motor_torque*factor;
	}
}

void ChShaftsMotor::ConstraintsLoadJacobians()
{
	if (motor_mode != MOT_MODE_TORQUE)
	{
		constraint.Get_Cq_a()->SetElement(0,0, 1);
		constraint.Get_Cq_b()->SetElement(0,0, -1);  
	}
}
 

void ChShaftsMotor::ConstraintsFetch_react(double factor)
{
	// From constraints to react vector:
	if (motor_mode != MOT_MODE_TORQUE)
		this->motor_torque = constraint.Get_l_i() * factor;  
}

// Following functions are for exploiting the contact persistence

void  ChShaftsMotor::ConstraintsLiLoadSuggestedSpeedSolution()
{
	if (motor_mode != MOT_MODE_TORQUE)
		constraint.Set_l_i(this->cache_li_speed);
}

void  ChShaftsMotor::ConstraintsLiLoadSuggestedPositionSolution()
{
	if (motor_mode != MOT_MODE_TORQUE)
		constraint.Set_l_i(this->cache_li_pos);
}

void  ChShaftsMotor::ConstraintsLiFetchSuggestedSpeedSolution()
{
	if (motor_mode != MOT_MODE_TORQUE)
		this->cache_li_speed = (float)constraint.Get_l_i();
}

void  ChShaftsMotor::ConstraintsLiFetchSuggestedPositionSolution()
{
	if (motor_mode != MOT_MODE_TORQUE)
		this->cache_li_pos =  (float)constraint.Get_l_i();
}



//////// FILE I/O

void ChShaftsMotor::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChShaftsCouple::StreamOUT(mstream);

		// stream out all member data
	mstream << this->motor_torque;
	mstream << (int)this->motor_mode;
	mstream << this->motor_set_rot;
	mstream << this->motor_set_rot_dt;
	
}

void ChShaftsMotor::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChShaftsCouple::StreamIN(mstream);

		// deserialize class
	int ifoo;
	mstream >> this->motor_torque;
	mstream >> ifoo; this->motor_mode = (ChShaftsMotor::eCh_shaftsmotor_mode)ifoo;
	mstream >> this->motor_set_rot;
	mstream >> this->motor_set_rot_dt;
}






} // END_OF_NAMESPACE____


/////////////////////
