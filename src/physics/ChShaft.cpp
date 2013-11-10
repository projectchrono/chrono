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
//   ChShaft.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChShaft.h"
#include "physics/ChSystem.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaft> a_registration_ChShaft;



//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SHAFTS


ChShaft::ChShaft ()
{
	this->torque = 0;
	this->system = 0;
	this->pos = 0;
	this->pos_dt = 0;
	this->pos_dtdt = 0;
	this->inertia = 1;
	this->fixed = false;
	this->limitspeed = false;

	SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID

	max_speed = 10.f;

	sleep_time = 0.6f;
	sleep_starttime = 0;
	sleep_minspeed = 0.1f;
	sleep_minwvel = 0.04f;
	this->sleeping = false;
	SetUseSleeping(true); 

	//variables.SetUserData((void*)this);
}



ChShaft::~ChShaft ()
{
	
}

void ChShaft::Copy(ChShaft* source)
{
		// copy the parent class data...
	ChPhysicsItem::Copy(source);

	this->torque = source->torque;
	this->system = source->system;
	this->pos = source->pos;
	this->pos_dt = source->pos_dt;
	this->pos_dtdt = source->pos_dtdt;
	this->inertia = source->inertia;
	this->fixed = source->fixed;
	this->sleeping = source->sleeping;
	this->limitspeed = source->limitspeed;

	this->system = source->system;

	variables = source->variables;

	max_speed = source->max_speed;

	sleep_time = source->sleep_time;
	sleep_starttime = source->sleep_starttime;
	sleep_minspeed = source->sleep_minspeed;
	sleep_minwvel = source->sleep_minwvel;
}



void ChShaft::SetInertia (double newJ) 
{ 
	assert(newJ >0.);
	if (newJ<=0.) 
		return;
	this->inertia = newJ; 
	this->variables.GetMass().SetElementN(0, newJ); 
	this->variables.GetInvMass().SetElementN(0, (1.0/newJ)); 
}



//// 
void ChShaft::InjectVariables(ChLcpSystemDescriptor& mdescriptor)
{	
	this->variables.SetDisabled(!this->IsActive());

	mdescriptor.InsertVariables(&this->variables);
}


void ChShaft::VariablesFbReset()
{
	this->variables.Get_fb().FillElem(0.0);
}

void ChShaft::VariablesFbLoadForces(double factor)
{
	// add applied torques to 'fb' vector
	this->variables.Get_fb().ElementN(0) += this->torque * factor;
}

void ChShaft::VariablesFbIncrementMq()
{
	this->variables.Compute_inc_Mb_v(this->variables.Get_fb(), this->variables.Get_qb());
}

void ChShaft::VariablesQbLoadSpeed()
{
	// set current speed in 'qb', it can be used by the LCP solver when working in incremental mode
	this->variables.Get_qb().SetElement(0,0, pos_dt);
}


void ChShaft::VariablesQbSetSpeed(double step)
{
	double old_dt = this->pos_dt;

	// from 'qb' vector, sets body speed, and updates auxiliary data
	this->pos_dt =  this->variables.Get_qb().GetElement(0,0);

	// apply limits (if in speed clamping mode) to speeds.
	ClampSpeed(); 

	// Compute accel. by BDF (approximate by differentiation);
	if (step)
	{
		pos_dtdt = ( (this->pos_dt - old_dt)  / step);
	}
}

void ChShaft::VariablesQbIncrementPosition(double dt_step)
{
	if (!this->IsActive()) 
		return;

	// Updates position with incremental action of speed contained in the
	// 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

	double newspeed = variables.Get_qb().GetElement(0,0);

	// ADVANCE POSITION: pos' = pos + dt * vel
	this->pos =  this->pos + newspeed * dt_step;
}



void ChShaft::SetNoSpeedNoAcceleration()
{
	this->pos_dt = 0;
	this->pos_dtdt = 0;
}


////
void ChShaft::ClampSpeed()
{
	if (this->GetLimitSpeed())
	{
		if (this->pos_dt > max_speed)
			this->pos_dt = max_speed;
		if (this->pos_dt < -max_speed)
			this->pos_dt = -max_speed;
	}
}




bool ChShaft::TrySleeping()
{
	if (this->GetUseSleeping())
	{	
		if (this->GetSleeping()) 
			return true;

		if ( fabs(this->pos_dt) < this->sleep_minspeed )
		{
				if ((this->GetChTime() - this->sleep_starttime) > this->sleep_time)
				{
					SetSleeping(true);
					return true;
				}
		}
		else
		{
			this->sleep_starttime = float(this->GetChTime());
		}
	}
	return false;
}



void ChShaft::Update (double mytime)
{
		// Update parent class too
	ChPhysicsItem::Update(mytime);

		// Class update

	//TrySleeping();			// See if the body can fall asleep; if so, put it to sleeping 
	ClampSpeed();			// Apply limits (if in speed clamping mode) to speeds.
}


//////// FILE I/O

void ChShaft::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChPhysicsItem::StreamOUT(mstream);

		// stream out all member data
	double dfoo;
	mstream << this->torque;
	mstream << this->pos;
	mstream << this->pos_dt;
	mstream << this->pos_dtdt;
	mstream << this->inertia;
	mstream << this->GetShaftFixed();

	mstream << max_speed;
	mstream << sleep_time;
	dfoo=(double)sleep_starttime;	mstream << dfoo;
	mstream << sleep_minspeed;
	mstream << sleep_minwvel;

	mstream << fixed;
	mstream << limitspeed;
	mstream << sleeping;
	mstream << use_sleeping;
}

void ChShaft::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChPhysicsItem::StreamIN(mstream);

	double dfoo;
	bool   bfoo;
	mstream >> this->torque;
	mstream >> this->pos;
	mstream >> this->pos_dt;
	mstream >> this->pos_dtdt;
	mstream >> this->inertia;
	mstream >> bfoo; this->SetShaftFixed(bfoo);

	mstream >> max_speed;
	mstream >> sleep_time;
	mstream >> dfoo;		sleep_starttime= (float)dfoo;
	mstream >> sleep_minspeed;
	mstream >> sleep_minwvel;
	
	mstream >> fixed;
	mstream >> limitspeed;
	mstream >> sleeping;
	mstream >> use_sleeping;
}





} // END_OF_NAMESPACE____


/////////////////////
