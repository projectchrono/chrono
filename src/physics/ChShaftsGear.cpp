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
//   ChShaftsGear.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChShaftsGear.h"
#include "physics/ChSystem.h"
#include "physics/ChShaft.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaftsGear> a_registration_ChShaftsGear;



//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SHAFTS


ChShaftsGear::ChShaftsGear ()
{
	this->ratio = 1;
	this->torque_react= 0;
	this->cache_li_speed =0.f;
	this->cache_li_pos = 0.f;

	SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID

	//variables.SetUserData((void*)this);
}


ChShaftsGear::~ChShaftsGear ()
{
	
}

void ChShaftsGear::Copy(ChShaftsGear* source)
{
		// copy the parent class data...
	ChShaftsCouple::Copy(source);

		// copy class data
	ratio = source->ratio;
	torque_react = source->torque_react;
	cache_li_speed = source->cache_li_speed;
	cache_li_pos = source->cache_li_pos;
}


int ChShaftsGear::Initialize(ChSharedPtr<ChShaft> mshaft1, ChSharedPtr<ChShaft> mshaft2)
{
	// Parent initialization
	if (!ChShaftsCouple::Initialize(mshaft1, mshaft2)) return false;

	ChShaft* mm1 = mshaft1.get_ptr();
	ChShaft* mm2 = mshaft2.get_ptr();

	this->constraint.SetVariables(&mm1->Variables(), &mm2->Variables());

	this->SetSystem(this->shaft1->GetSystem());
	return true;
}


void ChShaftsGear::Update (double mytime)
{
		// Inherit time changes of parent class
	ChShaftsCouple::Update(mytime);
	
		// update class data
	// ...
}




////////// LCP INTERFACES ////


void ChShaftsGear::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	//if (!this->IsActive())
	//	return;

	mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsGear::ConstraintsBiReset()
{
	constraint.Set_b_i(0.);
}
 
void ChShaftsGear::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	//if (!this->IsActive())
	//	return;

	double res = 0; // no residual

	constraint.Set_b_i(constraint.Get_b_i() +  factor * res);
}

void ChShaftsGear::ConstraintsBiLoad_Ct(double factor)
{
	//if (!this->IsActive())
	//	return;

	// nothing
}

/*
void ChShaftsGear::ConstraintsFbLoadForces(double factor)
{
	// no forces
}
*/

void ChShaftsGear::ConstraintsLoadJacobians()
{
		// compute jacobians
	constraint.Get_Cq_a()->SetElement(0,0, (float)this->ratio);
	constraint.Get_Cq_b()->SetElement(0,0, -1);  
}
 

void ChShaftsGear::ConstraintsFetch_react(double factor)
{
	// From constraints to react vector:
	this->torque_react = constraint.Get_l_i() * factor;  
}

// Following functions are for exploiting the contact persistence

void  ChShaftsGear::ConstraintsLiLoadSuggestedSpeedSolution()
{
	constraint.Set_l_i(this->cache_li_speed);
}

void  ChShaftsGear::ConstraintsLiLoadSuggestedPositionSolution()
{
	constraint.Set_l_i(this->cache_li_pos);
}

void  ChShaftsGear::ConstraintsLiFetchSuggestedSpeedSolution()
{
	this->cache_li_speed = (float)constraint.Get_l_i();
}

void  ChShaftsGear::ConstraintsLiFetchSuggestedPositionSolution()
{
	this->cache_li_pos =  (float)constraint.Get_l_i();
}



//////// FILE I/O

void ChShaftsGear::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChShaftsCouple::StreamOUT(mstream);

		// stream out all member data
	mstream << this->ratio;
	
}

void ChShaftsGear::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChShaftsCouple::StreamIN(mstream);

		// deserialize class
	mstream >> this->ratio;
	
}






} // END_OF_NAMESPACE____


/////////////////////
