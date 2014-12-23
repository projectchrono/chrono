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
//   ChShaftsClutch.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChShaftsClutch.h"
#include "physics/ChSystem.h"
#include "physics/ChShaft.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaftsClutch> a_registration_ChShaftClutch;



//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR CLUTCH


ChShaftsClutch::ChShaftsClutch ()
{
	this->maxT =  1.;
	this->minT = -1.;
	this->modulation = 1.0;

	this->torque_react= 0;
	this->cache_li_speed =0.f;
	this->cache_li_pos = 0.f;

	SetIdentifier(GetUniqueIntID()); // mark with unique ID

	//variables.SetUserData((void*)this);
}


ChShaftsClutch::~ChShaftsClutch ()
{
	
}

void ChShaftsClutch::Copy(ChShaftsClutch* source)
{
		// copy the parent class data...
	ChShaftsCouple::Copy(source);

		// copy class data
	maxT = source->maxT;
	minT = source->minT;
	modulation = source->modulation;

	torque_react = source->torque_react;
	cache_li_speed = source->cache_li_speed;
	cache_li_pos = source->cache_li_pos;
}


bool ChShaftsClutch::Initialize(ChSharedPtr<ChShaft> mshaft1, ChSharedPtr<ChShaft> mshaft2)
{
	// parent class initialization
	if (!ChShaftsCouple::Initialize(mshaft1, mshaft2)) return false;

	ChShaft* mm1 = mshaft1.get_ptr();
	ChShaft* mm2 = mshaft2.get_ptr();

	this->constraint.SetVariables(&mm1->Variables(), &mm2->Variables());

	this->SetSystem(this->shaft1->GetSystem());
	return true;
}


void ChShaftsClutch::Update (double mytime)
{
		// Inherit time changes of parent class
	ChShaftsCouple::Update(mytime);
	
		// update class data
	// ...
}


void  ChShaftsClutch::SetTorqueLimit(double ml, double mu)
{
	this->minT = ml;
	this->maxT = mu;
}



//// STATE BOOKKEEPING FUNCTIONS

 

void ChShaftsClutch::IntLoadResidual_CqL(
					const unsigned int off_L,	 ///< offset in L multipliers
					ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*Cq'*L 
					const ChVectorDynamic<>& L,  ///< the L vector 
					const double c				 ///< a scaling factor
					)
{
	constraint.MultiplyTandAdd(R, L(off_L) *c);
}

void ChShaftsClutch::IntLoadConstraint_C(
					const unsigned int off_L,	 ///< offset in Qc residual
					ChVectorDynamic<>& Qc,		 ///< result: the Qc residual, Qc += c*C 
					const double c,				 ///< a scaling factor
					bool do_clamp,				 ///< apply clamping to c*C?
					double recovery_clamp		 ///< value for min/max clamping of c*C
					)
{
	double res = 0; // no residual anyway! allow drifting...

	double cnstr_violation = c * res;

	if (do_clamp) {
		cnstr_violation = ChMin(ChMax(cnstr_violation, -recovery_clamp), recovery_clamp);
	}

	Qc(off_L) += cnstr_violation;
}

void ChShaftsClutch::IntToLCP(
					const unsigned int off_v,			///< offset in v, R
					const ChStateDelta& v,
					const ChVectorDynamic<>& R,
					const unsigned int off_L,			///< offset in L, Qc
					const ChVectorDynamic<>& L,
					const ChVectorDynamic<>& Qc
					)
{
	constraint.Set_l_i(L(off_L));

	constraint.Set_b_i(Qc(off_L));
}

void ChShaftsClutch::IntFromLCP(
					const unsigned int off_v,			///< offset in v
					ChStateDelta& v,
					const unsigned int off_L,			///< offset in L
					ChVectorDynamic<>& L
					)
{
	L(off_L)=constraint.Get_l_i();
}


////////// LCP INTERFACES ////


void ChShaftsClutch::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	//if (!this->IsActive())
	//	return;

	mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsClutch::ConstraintsBiReset()
{
	constraint.Set_b_i(0.);
}
 
void ChShaftsClutch::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	//if (!this->IsActive())
	//	return;

	double res = 0; // no residual

	constraint.Set_b_i(constraint.Get_b_i() +  factor * res);
}

void ChShaftsClutch::ConstraintsBiLoad_Ct(double factor)
{
	//if (!this->IsActive())
	//	return;

	// nothing
}


void ChShaftsClutch::ConstraintsFbLoadForces(double factor)
{
	// no forces

		// compute jacobians
	double m_dt = factor;

	this->constraint.SetBoxedMinMax(m_dt*this->minT*this->modulation, 
									m_dt*this->maxT*this->modulation);
}


void ChShaftsClutch::ConstraintsLoadJacobians()
{
	constraint.Get_Cq_a()->SetElement(0,0,  1.0);
	constraint.Get_Cq_b()->SetElement(0,0, -1.0); 
}
 

void ChShaftsClutch::ConstraintsFetch_react(double factor)
{
	// From constraints to react vector:
	this->torque_react = constraint.Get_l_i() * factor;  
}

// Following functions are for exploiting the contact persistence

void  ChShaftsClutch::ConstraintsLiLoadSuggestedSpeedSolution()
{
	constraint.Set_l_i(this->cache_li_speed);
}

void  ChShaftsClutch::ConstraintsLiLoadSuggestedPositionSolution()
{
	constraint.Set_l_i(this->cache_li_pos);
}

void  ChShaftsClutch::ConstraintsLiFetchSuggestedSpeedSolution()
{
	this->cache_li_speed = (float)constraint.Get_l_i();
}

void  ChShaftsClutch::ConstraintsLiFetchSuggestedPositionSolution()
{
	this->cache_li_pos =  (float)constraint.Get_l_i();
}



//////// FILE I/O

void ChShaftsClutch::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChShaftsCouple::StreamOUT(mstream);

		// stream out all member data
	mstream << this->maxT;
	mstream << this->minT;
	mstream << this->modulation;
	
}

void ChShaftsClutch::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChShaftsCouple::StreamIN(mstream);

		// deserialize class
	mstream >> this->maxT;
	mstream >> this->minT;
	mstream >> this->modulation;
	
}






} // END_OF_NAMESPACE____


/////////////////////
