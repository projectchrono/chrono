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



#include "physics/ChShaftsGearboxAngled.h"
#include "physics/ChSystem.h"
#include "physics/ChShaft.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaftsGearboxAngled> a_registration_ChShaftsGearboxAngled;





ChShaftsGearboxAngled::ChShaftsGearboxAngled ()
{
	this->t0=1;

	SetTransmissionRatio(1);

	this->torque_react= 0;
	this->cache_li_speed =0.f;
	this->cache_li_pos = 0.f;

	this->shaft1 = 0;
	this->shaft2 = 0;
	this->body = 0;

	this->shaft_dir1 = VECT_X;
	this->shaft_dir2 = VECT_X;

	SetIdentifier(GetUniqueIntID()); // mark with unique ID

}


ChShaftsGearboxAngled::~ChShaftsGearboxAngled ()
{
	
}

void ChShaftsGearboxAngled::Copy(ChShaftsGearboxAngled* source)
{
		// copy the parent class data...
	ChPhysicsItem::Copy(source);

		// copy class data
	t0 = source->t0;
	shaft_dir1 = source->shaft_dir1;
	shaft_dir2 = source->shaft_dir2;

	torque_react = source->torque_react;
	cache_li_speed = source->cache_li_speed;
	cache_li_pos = source->cache_li_pos;
	this->shaft1 = 0;
	this->shaft2 = 0;
	this->body = 0;
	
}


int ChShaftsGearboxAngled::Initialize(ChSharedPtr<ChShaft> mshaft1, ///< first (input) shaft to join
						   ChSharedPtr<ChShaft> mshaft2, ///< second  (output) shaft to join
						   ChSharedPtr<ChBodyFrame>  mbody,  ///< 3D body to use as truss (also carrier, if rotates as in planetary gearboxes) 
						   ChVector<>& mdir1,			 ///< the direction of the first shaft on 3D body defining the gearbox truss
						   ChVector<>& mdir2			 ///< the direction of the first shaft on 3D body defining the gearbox truss
						   )
{
	ChShaft* mm1 = mshaft1.get_ptr();
	ChShaft* mm2 = mshaft2.get_ptr();
	ChBodyFrame* mm3 = mbody.get_ptr();
	assert(mm1 && mm2 && mm3);
	assert(mm1 != mm2);
	assert((mm1->GetSystem() == mm2->GetSystem() ) );

	this->shaft1 = mm1;
	this->shaft2 = mm2;
	this->body = mm3;
	this->shaft_dir1 = mdir1;
	this->shaft_dir2 = mdir2;

	this->constraint.SetVariables(&mm1->Variables(), &mm2->Variables(), &mm3->Variables());

	this->SetSystem(this->shaft1->GetSystem());
	return true;
}


void ChShaftsGearboxAngled::Update (double mytime)
{
		// Inherit time changes of parent class
	ChPhysicsItem::Update(mytime);
	
		// update class data
	// ...
}




////////// LCP INTERFACES ////


void ChShaftsGearboxAngled::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	//if (!this->IsActive())
	//	return;

	mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsGearboxAngled::ConstraintsBiReset()
{
	constraint.Set_b_i(0.);
}
 
void ChShaftsGearboxAngled::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	//if (!this->IsActive())
	//	return;

	double res = 0; // no residual

	constraint.Set_b_i(constraint.Get_b_i() +  factor * res);
}

void ChShaftsGearboxAngled::ConstraintsBiLoad_Ct(double factor)
{
	//if (!this->IsActive())
	//	return;

	// nothing
}

/*
void ChShaftsGearboxAngled::ConstraintsFbLoadForces(double factor)
{
	// no forces
}
*/

void ChShaftsGearboxAngled::ConstraintsLoadJacobians()
{
		// compute jacobians
	constraint.Get_Cq_a()->SetElement(0,0, (float)this->t0);
	constraint.Get_Cq_b()->SetElement(0,0, (float)-1.0);

	//ChVector<> jacw = this->body->TrasformDirectionParentToLocal(this->t0*this->shaft_dir1 - this->shaft_dir2);
	ChVector<> jacw = (this->t0*this->shaft_dir1 - this->shaft_dir2);
	
	this->constraint.Get_Cq_c()->ElementN(0)=0;
	this->constraint.Get_Cq_c()->ElementN(1)=0;
	this->constraint.Get_Cq_c()->ElementN(2)=0;
	this->constraint.Get_Cq_c()->ElementN(3)=(float)jacw.x ;
	this->constraint.Get_Cq_c()->ElementN(4)=(float)jacw.y ;
	this->constraint.Get_Cq_c()->ElementN(5)=(float)jacw.z ;
}
 

void ChShaftsGearboxAngled::ConstraintsFetch_react(double factor)
{
	// From constraints to react vector:
	this->torque_react = constraint.Get_l_i() * factor;  
}

// Following functions are for exploiting the contact persistence

void  ChShaftsGearboxAngled::ConstraintsLiLoadSuggestedSpeedSolution()
{
	constraint.Set_l_i(this->cache_li_speed);
}

void  ChShaftsGearboxAngled::ConstraintsLiLoadSuggestedPositionSolution()
{
	constraint.Set_l_i(this->cache_li_pos);
}

void  ChShaftsGearboxAngled::ConstraintsLiFetchSuggestedSpeedSolution()
{
	this->cache_li_speed = (float)constraint.Get_l_i();
}

void  ChShaftsGearboxAngled::ConstraintsLiFetchSuggestedPositionSolution()
{
	this->cache_li_pos =  (float)constraint.Get_l_i();
}



//////// FILE I/O

void ChShaftsGearboxAngled::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChPhysicsItem::StreamOUT(mstream);

		// stream out all member data
	mstream << this->t0;
	mstream << this->shaft_dir1;
	mstream << this->shaft_dir2;
	
}

void ChShaftsGearboxAngled::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChPhysicsItem::StreamIN(mstream);

		// deserialize class
	mstream >> this->t0;
	mstream >> this->shaft_dir1;
	mstream >> this->shaft_dir2;
	
}






} // END_OF_NAMESPACE____


/////////////////////
