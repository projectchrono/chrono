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
//   ChShaftsBody.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChShaftsBody.h"
#include "physics/ChGlobal.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaftsBody> a_registration_ChShaftsBody;



//////////////////////////////////////
//////////////////////////////////////



ChShaftsBody::ChShaftsBody ()
{

	this->torque_react= 0;
	this->cache_li_speed =0.f;
	this->cache_li_pos = 0.f;

	this->shaft = 0;
	this->body = 0;
	this->shaft_dir = VECT_Z;

	SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID
}


ChShaftsBody::~ChShaftsBody ()
{
	
}

void ChShaftsBody::Copy(ChShaftsBody* source)
{
		// copy the parent class data...
	ChPhysicsItem::Copy(source);

		// copy class data

	torque_react = source->torque_react;
	cache_li_speed = source->cache_li_speed;
	cache_li_pos = source->cache_li_pos;
	this->shaft_dir = source->shaft_dir;
	this->shaft = 0;
	this->body = 0;
}


int ChShaftsBody::Initialize(ChSharedPtr<ChShaft>& mshaft, ChSharedPtr<ChBody>& mbody, ChVector<>& mdir)
{
	ChShaft* mm1 = mshaft.get_ptr();
	ChBody* mm2 = mbody.get_ptr();
	assert(mm1 && mm2);
	assert(mm1->GetSystem() == mm2->GetSystem());

	this->shaft = mm1;
	this->body = mm2;
	this->shaft_dir = Vnorm(mdir);

	this->constraint.SetVariables(&mm1->Variables(), &mm2->Variables());

	this->SetSystem(this->shaft->GetSystem());
	return true;
}


void ChShaftsBody::Update (double mytime)
{
		// Inherit time changes of parent class
	ChPhysicsItem::Update(mytime);
	
		// update class data
	// ...
}




////////// LCP INTERFACES ////


void ChShaftsBody::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	//if (!this->IsActive())
	//	return;

	mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsBody::ConstraintsBiReset()
{
	constraint.Set_b_i(0.);
}
 
void ChShaftsBody::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	//if (!this->IsActive())
	//	return;

	double res = 0; // no residual

	constraint.Set_b_i(constraint.Get_b_i() +  factor * res);
}

void ChShaftsBody::ConstraintsBiLoad_Ct(double factor)
{
	//if (!this->IsActive())
	//	return;

	// nothing
}


void ChShaftsBody::ConstraintsLoadJacobians()
{
		// compute jacobians
	ChVector<> jacw = this->body->Dir_World2Body(shaft_dir);

	this->constraint.Get_Cq_a()->ElementN(0)=-1;

	this->constraint.Get_Cq_b()->ElementN(0)=0;
	this->constraint.Get_Cq_b()->ElementN(1)=0;
	this->constraint.Get_Cq_b()->ElementN(2)=0;
	this->constraint.Get_Cq_b()->ElementN(3)=(float)jacw.x;
	this->constraint.Get_Cq_b()->ElementN(4)=(float)jacw.y;
	this->constraint.Get_Cq_b()->ElementN(5)=(float)jacw.z;
}
 

void ChShaftsBody::ConstraintsFetch_react(double factor)
{
	// From constraints to react vector:
	this->torque_react = constraint.Get_l_i() * factor;  
}

// Following functions are for exploiting the contact persistence

void  ChShaftsBody::ConstraintsLiLoadSuggestedSpeedSolution()
{
	constraint.Set_l_i(this->cache_li_speed);
}

void  ChShaftsBody::ConstraintsLiLoadSuggestedPositionSolution()
{
	constraint.Set_l_i(this->cache_li_pos);
}

void  ChShaftsBody::ConstraintsLiFetchSuggestedSpeedSolution()
{
	this->cache_li_speed = (float)constraint.Get_l_i();
}

void  ChShaftsBody::ConstraintsLiFetchSuggestedPositionSolution()
{
	this->cache_li_pos =  (float)constraint.Get_l_i();
}



//////// FILE I/O

void ChShaftsBody::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChPhysicsItem::StreamOUT(mstream);

		// stream out all member data
	mstream << this->shaft_dir;
	
}

void ChShaftsBody::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChPhysicsItem::StreamIN(mstream);

		// deserialize class
	mstream >> this->shaft_dir;
}






} // END_OF_NAMESPACE____


/////////////////////
