//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010, 2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChShaftsPlanetary.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChShaftsPlanetary.h"
#include "physics/ChSystem.h"
#include "physics/ChShaft.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaftsPlanetary> a_registration_ChShaftsPlanetary;



//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SHAFTS


ChShaftsPlanetary::ChShaftsPlanetary ()
{
	this->r1 = 1;
	this->r2 = 1;
	this->r3 = 1;

	this->torque_react= 0;
	this->cache_li_speed =0.f;
	this->cache_li_pos = 0.f;

	this->shaft1 = 0;
	this->shaft2 = 0;
	this->shaft3 = 0;

	SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID

	//variables.SetUserData((void*)this);
}


ChShaftsPlanetary::~ChShaftsPlanetary ()
{
	
}

void ChShaftsPlanetary::Copy(ChShaftsPlanetary* source)
{
		// copy the parent class data...
	ChPhysicsItem::Copy(source);

		// copy class data
	r1 = source->r1;
	r2 = source->r2;
	r3 = source->r3;

	torque_react = source->torque_react;
	cache_li_speed = source->cache_li_speed;
	cache_li_pos = source->cache_li_pos;
	this->shaft1 = 0;
	this->shaft2 = 0;
	this->shaft3 = 0;
}


int ChShaftsPlanetary::Initialize(ChSharedPtr<ChShaft> mshaft1, ChSharedPtr<ChShaft> mshaft2, ChSharedPtr<ChShaft> mshaft3)
{
	ChShaft* mm1 = mshaft1.get_ptr();
	ChShaft* mm2 = mshaft2.get_ptr();
	ChShaft* mm3 = mshaft3.get_ptr();
	assert(mm1 && mm2 && mm3);
	assert(mm1 != mm2);
	assert(mm1 != mm3);
	assert(mm3 != mm2);
	assert((mm1->GetSystem() == mm2->GetSystem()) &&( mm1->GetSystem() == mm3->GetSystem()) );

	this->shaft1 = mm1;
	this->shaft2 = mm2;
	this->shaft3 = mm3;

	this->constraint.SetVariables(&mm1->Variables(), &mm2->Variables(), &mm3->Variables());

	this->SetSystem(this->shaft1->GetSystem());
	return true;
}


void ChShaftsPlanetary::Update (double mytime)
{
		// Inherit time changes of parent class
	ChPhysicsItem::Update(mytime);
	
		// update class data
	// ...
}




////////// LCP INTERFACES ////


void ChShaftsPlanetary::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	//if (!this->IsActive())
	//	return;

	mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsPlanetary::ConstraintsBiReset()
{
	constraint.Set_b_i(0.);
}
 
void ChShaftsPlanetary::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	//if (!this->IsActive())
	//	return;

	double res = 0; // no residual

	constraint.Set_b_i(constraint.Get_b_i() +  factor * res);
}

void ChShaftsPlanetary::ConstraintsBiLoad_Ct(double factor)
{
	//if (!this->IsActive())
	//	return;

	// nothing
}

/*
void ChShaftsPlanetary::ConstraintsFbLoadForces(double factor)
{
	// no forces
}
*/

void ChShaftsPlanetary::ConstraintsLoadJacobians()
{
		// compute jacobians
	constraint.Get_Cq_a()->SetElement(0,0, (float)this->r1);
	constraint.Get_Cq_b()->SetElement(0,0, (float)this->r2);
	constraint.Get_Cq_c()->SetElement(0,0, (float)this->r3);
}
 

void ChShaftsPlanetary::ConstraintsFetch_react(double factor)
{
	// From constraints to react vector:
	this->torque_react = constraint.Get_l_i() * factor;  
}

// Following functions are for exploiting the contact persistence

void  ChShaftsPlanetary::ConstraintsLiLoadSuggestedSpeedSolution()
{
	constraint.Set_l_i(this->cache_li_speed);
}

void  ChShaftsPlanetary::ConstraintsLiLoadSuggestedPositionSolution()
{
	constraint.Set_l_i(this->cache_li_pos);
}

void  ChShaftsPlanetary::ConstraintsLiFetchSuggestedSpeedSolution()
{
	this->cache_li_speed = (float)constraint.Get_l_i();
}

void  ChShaftsPlanetary::ConstraintsLiFetchSuggestedPositionSolution()
{
	this->cache_li_pos =  (float)constraint.Get_l_i();
}



//////// FILE I/O

void ChShaftsPlanetary::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChPhysicsItem::StreamOUT(mstream);

		// stream out all member data
	mstream << this->r1;
	mstream << this->r2;
	mstream << this->r3;
	
}

void ChShaftsPlanetary::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChPhysicsItem::StreamIN(mstream);

		// deserialize class
	mstream >> this->r1;
	mstream >> this->r2;
	mstream >> this->r3;
	
}






} // END_OF_NAMESPACE____


/////////////////////
