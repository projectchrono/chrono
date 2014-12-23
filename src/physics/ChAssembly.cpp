//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChAssembly.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

    
#include <stdlib.h>
#include <algorithm>

#include "core/ChTransform.h"
#include "physics/ChAssembly.h"
#include "physics/ChGlobal.h"
#include "physics/ChSystem.h"

#include "physics/ChExternalObject.h"
#include "core/ChLinearAlgebra.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{


using namespace collision;
using namespace geometry;


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChAssembly> a_registration_ChAssembly;



//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SOLID BODIES


ChAssembly::ChAssembly ()
{
	do_collide = false;
	do_limit_speed = false;

	linklist.clear(); 
	bodylist.clear();

	max_speed = 0.5f;
	max_wvel  = 2.0f*float(CH_C_PI);

	SetIdentifier(GetUniqueIntID()); // mark with unique ID

}



ChAssembly::~ChAssembly ()
{
	RemoveAllBodies(); 
	RemoveAllLinks();
}

void ChAssembly::Copy(ChAssembly* source)
{
		// copy the parent class data...
	ChPhysicsItem::Copy(source);

	do_collide = source->do_collide;
	do_limit_speed = source->do_limit_speed;

	max_speed = source->max_speed;
	max_wvel  = source->max_wvel;
}

void ChAssembly::SetSystem(ChSystem* m_system)
{
	this->system=m_system;

	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->SetSystem(m_system);
	}
	
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->SetSystem(m_system);
	}
}

void ChAssembly::Clear()
{
	RemoveAllLinks();
	RemoveAllBodies();
}


void ChAssembly::RemoveAllBodies() 
{ 
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];

		// make sure to remove bodies from collision system before calling this.

		// nullify backward link to system
		Bpointer->SetSystem(0);	
		// this may delete the body, if none else's still referencing it..
		Bpointer->RemoveRef();
	}	
	bodylist.clear(); 
}


void ChAssembly::RemoveAllLinks() 
{ 

	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		// nullify backward link to system
		Lpointer->SetSystem(0);	
		// this may delete the link, if none else's still referencing it..
		Lpointer->RemoveRef();
	}	
	linklist.clear(); 
}

int ChAssembly::GetDOF()
{
	int ndof = 0;
					
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies		
	{
		ChBody* Bpointer = bodylist[ip];
		ndof+=Bpointer->GetDOF();
	}
	return ndof;
}

int ChAssembly::GetDOF_w()
{
	int ndof = 0;

	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies		
	{
		ChBody* Bpointer = bodylist[ip];
		ndof += Bpointer->GetDOF_w();
	}
	return ndof;
}

int ChAssembly::GetDOC_c()
{
	int ndoc=0;

	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		ndoc   += Lpointer->GetDOC_c();
	}
	return ndoc;
}

int ChAssembly::GetDOC_d()
{
	int ndoc=0;

	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		ndoc   += Lpointer->GetDOC_d();
	}
	return ndoc;
}


//// STATE BOOKKEEPING FUNCTIONS

void ChAssembly::IntStateGather(
					const unsigned int off_x,		///< offset in x state vector
					ChState& x,						///< state vector, position part
					const unsigned int off_v,		///< offset in v state vector
					ChStateDelta& v,				///< state vector, speed part
					double& T)						///< time
{
	unsigned int local_off_x=0;
	unsigned int local_off_v=0;
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->IntStateGather(off_x+local_off_x, x, off_v+local_off_v, v, T);
		local_off_x += Bpointer->GetDOF();
		local_off_v += Bpointer->GetDOF_w();
	}
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->IntStateGather(off_x+local_off_x, x, off_v+local_off_v, v, T);
		local_off_x += Lpointer->GetDOF();
		local_off_v += Lpointer->GetDOF_w();
	}
}

void ChAssembly::IntStateScatter(
					const unsigned int off_x,		///< offset in x state vector
					const ChState& x,				///< state vector, position part
					const unsigned int off_v,		///< offset in v state vector
					const ChStateDelta& v,			///< state vector, speed part
					const double T) 				///< time
{
	unsigned int local_off_x=0;
	unsigned int local_off_v=0;
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->IntStateScatter(off_x+local_off_x, x, off_v+local_off_v, v, T);
		local_off_x += Bpointer->GetDOF();
		local_off_v += Bpointer->GetDOF_w();
	}
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->IntStateScatter(off_x+local_off_x, x, off_v+local_off_v, v, T);
		local_off_x += Lpointer->GetDOF();
		local_off_v += Lpointer->GetDOF_w();
	}	
}

void ChAssembly::IntStateIncrement(
					const unsigned int off_x,		///< offset in x state vector
					ChState& x_new,					///< state vector, position part, incremented result
					const ChState& x,				///< state vector, initial position part
					const unsigned int off_v,		///< offset in v state vector
					const ChStateDelta& Dv)  		///< state vector, increment
{
	unsigned int local_off_x=0;
	unsigned int local_off_v=0;
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->IntStateIncrement(off_x+local_off_x, x_new, x, off_v+local_off_v, Dv);
		local_off_x += Bpointer->GetDOF();
		local_off_v += Bpointer->GetDOF_w();
	}
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->IntStateIncrement(off_x+local_off_x, x_new, x, off_v+local_off_v, Dv);
		local_off_x += Lpointer->GetDOF();
		local_off_v += Lpointer->GetDOF_w();
	}
}
 
void ChAssembly::IntLoadResidual_F(
					const unsigned int off,		 ///< offset in R residual
					ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*F 
					const double c				 ///< a scaling factor
					)
{
	unsigned int local_off_v=0;
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->IntLoadResidual_F(off+local_off_v, R, c);
		local_off_v += Bpointer->GetDOF_w();
	}
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->IntLoadResidual_F(off+local_off_v, R, c);
		local_off_v += Lpointer->GetDOF_w();
	}	
}


void ChAssembly::IntLoadResidual_Mv(
					const unsigned int off,		 ///< offset in R residual
					ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*M*v 
					const ChVectorDynamic<>& w,  ///< the w vector 
					const double c				 ///< a scaling factor
					)
{
	unsigned int local_off_v=0;
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->IntLoadResidual_Mv(off+local_off_v, R, w, c);
		local_off_v += Bpointer->GetDOF_w();
	}
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->IntLoadResidual_Mv(off+local_off_v, R, w, c);
		local_off_v += Lpointer->GetDOF_w();
	}
}

void ChAssembly::IntLoadResidual_CqL(
					const unsigned int off_L,	 ///< offset in L multipliers
					ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*Cq'*L 
					const ChVectorDynamic<>& L,  ///< the L vector 
					const double c				 ///< a scaling factor
					)
{
	unsigned int local_off_L=0;
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->IntLoadResidual_CqL(off_L+local_off_L, R, L, c);
		local_off_L += Bpointer->GetDOC();
	}
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->IntLoadResidual_CqL(off_L+local_off_L, R, L, c);
		local_off_L += Lpointer->GetDOC();
	}
}

void ChAssembly::IntLoadConstraint_C(
					const unsigned int off_L,		 ///< offset in Qc residual
					ChVectorDynamic<>& Qc,		 ///< result: the Qc residual, Qc += c*C 
					const double c,				 ///< a scaling factor
					bool do_clamp,				 ///< apply clamping to c*C?
					double recovery_clamp		 ///< value for min/max clamping of c*C
					)
{
	unsigned int local_off_L=0;
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->IntLoadConstraint_C(off_L+local_off_L, Qc, c, do_clamp, recovery_clamp);
		local_off_L += Bpointer->GetDOC();
	}
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->IntLoadConstraint_C(off_L+local_off_L, Qc, c, do_clamp, recovery_clamp);
		local_off_L += Lpointer->GetDOC();
	}
}


void ChAssembly::IntLoadConstraint_Ct(
					const unsigned int off_L,		 ///< offset in Qc residual
					ChVectorDynamic<>& Qc,		 ///< result: the Qc residual, Qc += c*Ct 
					const double c			 ///< a scaling factor
					)
{
	unsigned int local_off_L=0;
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->IntLoadConstraint_Ct(off_L+local_off_L, Qc, c);
		local_off_L += Bpointer->GetDOC();
	}
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->IntLoadConstraint_Ct(off_L+local_off_L, Qc, c);
		local_off_L += Lpointer->GetDOC();
	}
}


void ChAssembly::IntToLCP(
					const unsigned int off_v,			///< offset in v, R
					const ChStateDelta& v,
					const ChVectorDynamic<>& R,
					const unsigned int off_L,			///< offset in L, Qc
					const ChVectorDynamic<>& L,
					const ChVectorDynamic<>& Qc
					)
{
	unsigned int local_off_L=0;
	unsigned int local_off_v=0;
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->IntToLCP(off_v+local_off_v, v, R, off_L+local_off_L, L, Qc);
		local_off_L += Bpointer->GetDOC();
		local_off_v += Bpointer->GetDOF_w();
	}
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->IntToLCP(off_v+local_off_v, v, R, off_L+local_off_L, L, Qc);
		local_off_L += Lpointer->GetDOC();
		local_off_v += Lpointer->GetDOF_w();
	}
}

void ChAssembly::IntFromLCP(
					const unsigned int off_v,			///< offset in v
					ChStateDelta& v,
					const unsigned int off_L,			///< offset in L
					ChVectorDynamic<>& L
					)
{
	unsigned int local_off_L=0;
	unsigned int local_off_v=0;
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->IntFromLCP(off_v+local_off_v, v, off_L+local_off_L, L);
		local_off_L += Bpointer->GetDOC();
		local_off_v += Bpointer->GetDOF_w();
	}
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->IntFromLCP(off_v+local_off_v, v, off_L+local_off_L, L);
		local_off_L += Lpointer->GetDOC();
		local_off_v += Lpointer->GetDOF_w();
	}
}



//// 
void ChAssembly::InjectVariables(ChLcpSystemDescriptor& mdescriptor)
{	
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->InjectVariables(mdescriptor);
	}
}


void ChAssembly::VariablesFbReset()
{
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->VariablesFbReset();
	}
}

void ChAssembly::VariablesFbLoadForces(double factor)
{
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->VariablesFbLoadForces(factor);
	}
}

void ChAssembly::VariablesFbIncrementMq()
{
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->VariablesFbIncrementMq();
	}
}

void ChAssembly::VariablesQbLoadSpeed()
{
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->VariablesQbLoadSpeed();
	}
}


void ChAssembly::VariablesQbSetSpeed(double step)
{
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->VariablesQbSetSpeed(step);
	}
}

void ChAssembly::VariablesQbIncrementPosition(double dt_step)
{
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->VariablesQbIncrementPosition(dt_step);
	}
}

void ChAssembly::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->InjectConstraints(mdescriptor);
	}
}

void ChAssembly::ConstraintsBiReset()
{
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->ConstraintsBiReset();
	}
}

void ChAssembly::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) 
{
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
	}
}

void ChAssembly::ConstraintsBiLoad_Ct(double factor)
{
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->ConstraintsBiLoad_Ct(factor);
	}
}

void ChAssembly::ConstraintsBiLoad_Qc(double factor)
{
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->ConstraintsBiLoad_Qc(factor);
	}
}

void ChAssembly::ConstraintsFbLoadForces(double factor)
{
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->ConstraintsFbLoadForces(factor);
	}
}

void ChAssembly::ConstraintsLoadJacobians()
{
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->ConstraintsLoadJacobians();
	}
}

void ChAssembly::ConstraintsLiLoadSuggestedSpeedSolution()
{
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->ConstraintsLiLoadSuggestedSpeedSolution();
	}
}

void ChAssembly::ConstraintsLiLoadSuggestedPositionSolution()
{
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->ConstraintsLiLoadSuggestedPositionSolution();
	}
}

void ChAssembly::ConstraintsLiFetchSuggestedSpeedSolution()
{
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->ConstraintsLiFetchSuggestedSpeedSolution();
	}
}

void ChAssembly::ConstraintsLiFetchSuggestedPositionSolution()
{
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->ConstraintsLiFetchSuggestedPositionSolution();
	}
}

void ChAssembly::ConstraintsFetch_react(double factor)
{
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->ConstraintsFetch_react(factor);
	}
}


void ChAssembly::SetNoSpeedNoAcceleration()
{
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->SetNoSpeedNoAcceleration();
	}
}


////
void ChAssembly::ClampSpeed()
{
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->ClampSpeed();
	}
}


							// UpdateALL updates the state and time
							// of the object AND the dependant (linked)
							// markers and forces.

void ChAssembly::Update()
{
	ChAssembly::Update(this->GetChTime());
}



							// As before, but keeps the current state.
							// Mostly used for world reference body.
void ChAssembly::Update (double mytime)
{
	ChTime = mytime;
	ClampSpeed();			// Apply limits (if in speed clamping mode) to speeds.

	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->Update(mytime);
	}
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
		Lpointer->Update(mytime);
	}
}

void ChAssembly::AddBody (ChSharedPtr<ChBody> newbody)
{
	assert(std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), newbody.get_ptr())==bodylist.end());
	assert(newbody->GetSystem()==0); // should remove from other system before adding here

	newbody->AddRef();
	newbody->SetSystem (this->GetSystem());
	bodylist.push_back((newbody).get_ptr());

	// add to collision system too
	//if (newbody->GetCollide())
	//	newbody->AddCollisionModelsToSystem();
}

void ChAssembly::AddLink (ChLink* newlink)
{ 
	assert(std::find<std::vector<ChLink*>::iterator>(linklist.begin(), linklist.end(), newlink)==linklist.end());

	newlink->AddRef();
	newlink->SetSystem (this->GetSystem());
	linklist.push_back(newlink);
}

void ChAssembly::AddLink (ChSharedPtr<ChLink> newlink)
{
	AddLink(newlink.get_ptr());
}

void ChAssembly::RemoveBody (ChSharedPtr<ChBody> mbody)
{
	assert(std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr() )!=bodylist.end());

	// remove from collision system
	if (mbody->GetCollide())
		mbody->RemoveCollisionModelsFromSystem(); 
 
	// warning! linear time search, to erase pointer from container.
	bodylist.erase(std::find<std::vector<ChBody*>::iterator>(bodylist.begin(), bodylist.end(), mbody.get_ptr() ) );
	
	// nullify backward link to system
	mbody->SetSystem(0);
	// this may delete the body, if none else's still referencing it..
	mbody->RemoveRef();
}

// Faster than RemoveLink because it does not require the linear time search
std::vector<ChLink*>::iterator ChAssembly::RemoveLinkIter(std::vector<ChLink*>::iterator& mlinkiter)
{
	// nullify backward link to system
	(*mlinkiter)->SetSystem(0);
	// this may delete the link, if none else's still referencing it..
	(*mlinkiter)->RemoveRef();

	return linklist.erase(mlinkiter);
}

void ChAssembly::RemoveLink (ChSharedPtr<ChLink> mlink)
{
	assert(std::find<std::vector<ChLink*>::iterator>(linklist.begin(), linklist.end(), mlink.get_ptr() )!=linklist.end());

	// warning! linear time search, to erase pointer from container.
	linklist.erase(std::find<std::vector<ChLink*>::iterator>(linklist.begin(), linklist.end(), mlink.get_ptr()));

	// nullify backward link to system
	mlink->SetSystem(0);
	// this may delete the body, if none else's still referencing it..
	mlink->RemoveRef();
}

 
// collision stuff
void ChAssembly::SetCollide (bool mcoll)
{
	if (mcoll == do_collide) 
		return;

	if (mcoll)
	{
		SyncCollisionModels();
		this->do_collide=true;
		if (GetSystem())
		{
			AddCollisionModelsToSystem();
		}
	}
	else 
	{
		this->do_collide = false;
		if (GetSystem())
		{
			RemoveCollisionModelsFromSystem();
		}
	}
}

void ChAssembly::SyncCollisionModels()
{
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		Bpointer->GetCollisionModel()->SyncPosition();
	}
}

void ChAssembly::AddCollisionModelsToSystem() 
{
	assert(this->GetSystem());
	SyncCollisionModels();

	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		if (Bpointer->GetCollide())
			this->GetSystem()->GetCollisionSystem()->Add(Bpointer->GetCollisionModel());
	}
}

void ChAssembly::RemoveCollisionModelsFromSystem() 
{
	assert(this->GetSystem());

	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		if (Bpointer->GetCollide())
			this->GetSystem()->GetCollisionSystem()->Remove(Bpointer->GetCollisionModel());
	}
}



void ChAssembly::GetTotalAABB(ChVector<>& bbmin, ChVector<>& bbmax)
{
	//default infinite bb
	ChVector<> mmin(-1e200, -1e200, -1e200);
	ChVector<> mmax( 1e200,  1e200,  1e200);

	bool set = false;
	ChVector<> tmpMin;
	ChVector<> tmpMax;

	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
		if (Bpointer->GetCollisionModel())
		{
			Bpointer->GetCollisionModel()->GetAABB(tmpMin, tmpMax);
			if (!set)
			{
				mmin=tmpMin;
				mmax=tmpMax;
				set=true;
			}
			if (tmpMin.x<mmin.x)
				mmin.x=tmpMin.x;
			if (tmpMin.y<mmin.y)
				mmin.y=tmpMin.y;
			if (tmpMin.z<mmin.z)
				mmin.z=tmpMin.z;
			if (tmpMax.x>mmax.x)
				mmax.x=tmpMax.x;
			if (tmpMax.y>mmax.y)
				mmax.y=tmpMax.y;
			if (tmpMax.z>mmax.z)
				mmax.z=tmpMax.z;
		}
	}
	bbmin.Set(mmin.x, mmin.y, mmin.z);
	bbmax.Set(mmax.x, mmax.y, mmax.z);
}

void ChAssembly::Reference_LM_byID()
{
	std::vector< ChLink* > toremove;

	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];

		if (ChLinkMarkers* malink = ChDynamicCast(ChLinkMarkers, Lpointer))
		{
			ChSharedPtr<ChMarker> shm1 = SearchMarker(malink->GetMarkID1());
			ChSharedPtr<ChMarker> shm2 = SearchMarker(malink->GetMarkID2());
			ChMarker* mm1 = shm1.get_ptr();
			ChMarker* mm2 = shm1.get_ptr();
			malink->SetUpMarkers(mm1, mm2);
			if (mm1 && mm2)
			{
				Lpointer->SetValid(true);
			}
			else
			{
				Lpointer->SetValid(false);
				malink->SetUpMarkers(0, 0); // note: marker IDs are maintained
				toremove.push_back(Lpointer);
			}
		}

	}
	for (int ir = 0; ir < toremove.size(); ++ir)
	{
		ChSharedPtr<ChLink> mlink(toremove[ir]);
		toremove[ir]->AddRef(); // cause shared from raw pointer from vector cointainer

		RemoveLink(mlink);
	}
}

ChSharedPtr<ChMarker> ChAssembly::SearchMarker (int markID)
{
	ChMarker* candidate = NULL;
	ChMarker* res = NULL;

	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];

		res = ChContainerSearchFromID<ChMarker, std::vector<ChMarker*>::const_iterator>
			(markID,
			Bpointer->GetMarkerList().begin(),
			Bpointer->GetMarkerList().end());
		if (res != NULL)
		{
			res->AddRef(); // in that container pointers were not stored as ChSharedPtr, so this is needed..
			return (ChSharedPtr<ChMarker>(res));  // ..here I am not getting a new() data, but a reference to something created elsewhere		
		}
	}

	return (ChSharedPtr<ChMarker>()); // not found? return a void shared ptr.
}

//////// FILE I/O

void ChAssembly::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChPhysicsItem::StreamOUT(mstream);

		// stream out all member data
	mstream << do_collide;
	mstream << do_limit_speed;
	
	mstream << max_speed;
	mstream << max_wvel;

	// 2a) write how many bodies
	mstream << (int)bodylist.size();

	// 2b) write  bodies
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
			// write the body
		//Bpointer->StreamOUT(mstream);
		mstream.AbstractWriteAll(Bpointer);
		//mstream.AbstractWrite(Bpointer);
	}

	// 3a) write how many links
	mstream << (int)linklist.size();

	// 3b) write links links
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
			// Writethe link, using a special downcasting function Link_BinSave which saves also the
			// inheritance info, depending on link class inheritance from base Link*
		mstream.AbstractWrite(Lpointer);
	}
}

void ChAssembly::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChPhysicsItem::StreamIN(mstream);

	mstream >> do_collide;
	mstream >> do_limit_speed;
	
	mstream >> max_speed;
	mstream >> max_wvel;

	// 2a) read how many bodies
	int mnbodies = 0;
	mstream >> mnbodies;

	// 2b) read  bodies
	ChBody* newbody= NULL;
	for (int i= 0; i<mnbodies; i++)
	{
		//mstream.AbstractReadCreate(&newbody);
		mstream.AbstractReadAllCreate(&newbody);
		ChSharedPtr<ChBody> shitem(newbody);
		this->AddBody(shitem);
		/*
		ChSharedPtr<ChBody> newbody(new ChBody);
		this->AddBody(newbody);

		newbody->StreamIN(mstream);
		*/
	}

	// 3a) read how many links
	int mnlinks = 0;
	mstream >> mnlinks;

	// 3b) read  links
	ChLink* newlink= NULL;
	for (int j= 0; j<mnlinks; j++)
	{
			// read the link, using a special downcasting function Link_BinRead_Create which creates the
			// proper inherited object, depending on its class inheritance from base Link*

		mstream.AbstractReadCreate(&newlink);

		ChSharedPtr<ChLink> shlink(newlink);
		this->AddLink(shlink);
	}

	// 3c) Rebuild link pointers to markers
	this->Reference_LM_byID();
}




void ChAssembly::StreamOUTstate(ChStreamOutBinary& mstream)
{
	// Do not serialize parent classes and do not
	// implement versioning, because this must be efficient 
	// and will be used just for domain decomposition.
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
			// write the body + child markers + forces
		Bpointer->StreamOUTstate(mstream);
	}
}

void ChAssembly::StreamINstate(ChStreamInBinary& mstream)
{
	// Do not serialize parent classes and do not
	// implement versioning, because this must be efficient 
	// and will be used just for domain decomposition.
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
			// write the body + child markers + forces
		Bpointer->StreamINstate(mstream);
	}
}


#define CH_CHUNK_END_ASSEM 18881

int ChAssembly::StreamINall  (ChStreamInBinary& m_file)
{
	int mchunk = 0;
	ChBody* newbody= NULL;
	ChLink* newlink= NULL;

	// class version number
	int version = m_file.VersionRead();

	// 0) reset system to have no sub object child
	this->Clear();

	// 1) read system class data...
		// deserialize parent class too
	ChPhysicsItem::StreamIN(m_file);

	m_file >> do_collide;
	m_file >> do_limit_speed;

	m_file >> max_speed;
	m_file >> max_wvel;

	// 2a) read how many bodies
	int mnbodies = 0;
	m_file >> mnbodies;

	// 2b) read  bodies
	for (int i= 0; i<mnbodies; i++)
	{
		ChSharedPtr<ChBody> newbody(new ChBody);
		this->AddBody(newbody);

		if (!newbody->StreamINall(m_file)) 
			throw ChException("Cannot read body data");
	}

	// 3a) read how many links
	int mnlinks = 0;
	m_file >> mnlinks;

	// 3b) read  links
	for (int j= 0; j<mnlinks; j++)
	{
			// read the link, using a special downcasting function Link_BinRead_Create which creates the
			// proper inherited object, depending on its class inheritance from base Link*

		m_file.AbstractReadCreate(&newlink);
		if (!newlink) throw ChException("Cannot read link data");

		ChSharedPtr<ChLink> shlink(newlink);
		this->AddLink(shlink);
	}

	// 3c) Rebuild link pointers to markers
	this->Reference_LM_byID();

	m_file >> mchunk;

	if (mchunk != CH_CHUNK_END_ASSEM) return 0;

	return 1;
}


int ChAssembly::StreamOUTall  (ChStreamOutBinary& m_file)
{
	// class version number
	m_file.VersionWrite(1);

	// 1) write system class data...
		// serialize parent class too
	ChPhysicsItem::StreamOUT(m_file);

		// stream out all member data
	m_file << do_collide;
	m_file << do_limit_speed;

	m_file << max_speed;
	m_file << max_wvel;

	// 2a) write how many bodies
	m_file << (int)bodylist.size();

	// 2b) write  bodies
	for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
	{
		ChBody* Bpointer = bodylist[ip];
			// write the body + child markers + forces
		if (!Bpointer->StreamOUTall(m_file)) return 0;
	}

	// 3a) write how many links
	m_file << (int)linklist.size(); 

	// 3b) write links links
	for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
	{
		ChLink* Lpointer = linklist[ip];
			// Writethe link, using a special downcasting function Link_BinSave which saves also the
			// inheritance info, depending on link class inheritance from base Link*
		m_file.AbstractWrite(Lpointer);
	}

	m_file << (int)CH_CHUNK_END_ASSEM;

	return 1;
}

void ChAssembly::StreamOUT(ChStreamOutAscii& mstream)
{
	//***TO DO***
}


int ChAssembly::StreamOUTall  (ChStreamOutAscii& mstream) // dump rigidbody and childrens (markers.forces)
{
	//***TO DO***

	StreamOUT (mstream);			 // 1) dump the body attrs

	return 1;
}


} // END_OF_NAMESPACE____


/////////////////////
