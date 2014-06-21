//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChContactContainerNodes.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
#include "physics/ChContactContainerNodes.h"
#include "physics/ChSystem.h"
#include "physics/ChIndexedNodes.h"
#include "physics/ChNodeXYZ.h"
#include "physics/ChBody.h"
#include "lcp/ChLcpConstraintNodeContactN.h"
#include "collision/ChCModelBulletBody.h"
#include "collision/ChCModelBulletNode.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


using namespace collision;
using namespace geometry;





// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChContactContainerNodes> a_registration_ChContactContainerNodes;
 

ChContactContainerNodes::ChContactContainerNodes ()
{ 
	contactlist.clear();
	n_added = 0;

}


ChContactContainerNodes::~ChContactContainerNodes ()
{
	std::list<ChContactNode*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		delete (*itercontact);
		(*itercontact) = 0;
		++itercontact;
		//contactlist.erase(itercontact); //no! do later with clear(), all together
	}
	contactlist.clear();
}



void ChContactContainerNodes::Update (double mytime)
{
    // Inherit time changes of parent class, basically doing nothing :)
    ChContactContainerBase::Update(mytime);

}


void ChContactContainerNodes::RemoveAllContacts()
{
	std::list<ChContactNode*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		delete (*itercontact);
		(*itercontact) = 0;
		++itercontact;
		//contactlist.erase(itercontact); //no! do later with clear(), all together
	}
	contactlist.clear();

	lastcontact = contactlist.begin();
	n_added = 0;
}


void ChContactContainerNodes::BeginAddContact()
{
	lastcontact = contactlist.begin();
	n_added = 0;
}

void ChContactContainerNodes::EndAddContact()
{
	// remove contacts that are beyond last contact
	while (lastcontact != contactlist.end())
	{
		delete (*lastcontact);
		lastcontact = contactlist.erase(lastcontact);
	}
}


void ChContactContainerNodes::AddContact(const collision::ChCollisionInfo& mcontact)
{
	// Fetch the frames of that contact and other infos

	ChModelBulletBody* mmboA=0;
	ChModelBulletNode* mmnoB=0;
	bool swapped = false;

	if (mmnoB = dynamic_cast<ChModelBulletNode*>(mcontact.modelA))
	{
		mmboA = dynamic_cast<ChModelBulletBody*>(mcontact.modelB);
		swapped = true;
	}
	else
	{
		if (mmnoB = dynamic_cast<ChModelBulletNode*>(mcontact.modelB))
			mmboA = dynamic_cast<ChModelBulletBody*>(mcontact.modelA);
	}

	if (!(mmboA && mmnoB))
		return;	// bailout if no body-node or node-body case

	ChFrame<>*  frameA =0;
	ChVector<>* posB =0;
	bool fixedA = false;
	bool fixedB = false;
	float frictionA; //float frictionB;
	ChLcpVariablesBody* varA = 0;
	ChLcpVariablesNode* varB = 0;

	ChVector<> mvN  = mcontact.vN;
	ChVector<> mvpA = mcontact.vpA;
	ChVector<> mvpB = mcontact.vpB;
	if (swapped)
	{
		mvN  = -mcontact.vN;
		mvpA =  mcontact.vpB;
		mvpB =  mcontact.vpA;
	}

	frameA    = mmboA->GetBody();
	varA      =&mmboA->GetBody()->VariablesBody();
	//fixedA    = mmboA->GetBody()->GetBodyFixed();
	frictionA = mmboA->GetBody()->GetSfriction();
	
	// downcast
	ChSharedPtr<ChNodeXYZ> mnode ( mmnoB->GetNodes()->GetNode(mmnoB->GetNodeId()).DynamicCastTo<ChNodeXYZ>() );
	if (mnode.IsNull()) 
		return;

	posB      = &mnode->pos;
	varB      = (ChLcpVariablesNode*)&mnode->Variables();
	//fixedB    = mmnoB->GetNodes()->GetBodyFixed();
	//frictionB = mmnoB->GetNodes()->GetSfriction();

	// Compute default material-couple values

	ChMaterialCouple mat;
	mat.static_friction   =frictionA; //(frictionA     + frictionB)*0.5f;
	mat.rolling_friction  =0;
	mat.spinning_friction =0;

	// Launch the contact callback, if any, to set custom friction & material 
	// properties, if implemented by the user:

	if (this->add_contact_callback)
	{
		this->add_contact_callback->ContactCallback(mcontact, mat);
	}

	// %%%%%%% Create and add a ChContactNode object  %%%%%%%

	if (lastcontact != contactlist.end())
	{
		// reuse old contacts
		(*lastcontact)->Reset(		  mmboA,
									  mmnoB,
									  varA, varB,
									  frameA, posB,
									  mvpA,
									  mvpB, 
									  mvN,
									  mcontact.distance, 
									  mcontact.reaction_cache,
									  mat.static_friction);
		lastcontact++;
	}
	else
	{
		// add new contact
		ChContactNode* mc = new ChContactNode(mmboA,
									  mmnoB,
									  varA, varB,
									  frameA, posB,
									  mvpA,
									  mvpB, 
									  mvN,
									  mcontact.distance, 
									  mcontact.reaction_cache,
									  mat.static_friction);
		contactlist.push_back(mc);
		lastcontact = contactlist.end();
	}
	n_added++;
	
}



void ChContactContainerNodes::ReportAllContacts(ChReportContactCallback* mcallback)
{
	std::list<ChContactNode*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		bool proceed = mcallback->ReportContactCallback(
					(*itercontact)->GetContactP1(),
					(*itercontact)->GetContactP2(),
					*(*itercontact)->GetContactPlane(),
					(*itercontact)->GetContactDistance(),
					(*itercontact)->GetFriction(),
					(*itercontact)->GetContactForce(),
					VNULL, // no react torques
					(*itercontact)->GetModelA(), 
					(*itercontact)->GetModelB()  
					);
		if (!proceed) 
			break;
		++itercontact;
	}

}


////////// LCP INTERFACES ////


void ChContactContainerNodes::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	std::list<ChContactNode*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->InjectConstraints(mdescriptor);
		++itercontact;
	}
}

void ChContactContainerNodes::ConstraintsBiReset()
{
	std::list<ChContactNode*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsBiReset();
		++itercontact;
	}
}
 
void ChContactContainerNodes::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	std::list<ChContactNode*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
		++itercontact;
	}
}


void ChContactContainerNodes::ConstraintsLoadJacobians()
{
	// already loaded when ChContact objects are created
}
 

void ChContactContainerNodes::ConstraintsFetch_react(double factor)
{
	// From constraints to react vector:
	std::list<ChContactNode*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsFetch_react(factor);
		++itercontact;
	}
}


// Following functions are for exploiting the contact persistence


void  ChContactContainerNodes::ConstraintsLiLoadSuggestedSpeedSolution()
{
	// Fetch the last computed impulsive reactions from the persistent contact manifold (could
	// be used for warm starting the CCP speed solver):
	std::list<ChContactNode*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsLiLoadSuggestedSpeedSolution();
		++itercontact;
	}
}

void  ChContactContainerNodes::ConstraintsLiLoadSuggestedPositionSolution()
{
	// Fetch the last computed 'positional' reactions from the persistent contact manifold (could
	// be used for warm starting the CCP position stabilization solver):
	std::list<ChContactNode*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsLiLoadSuggestedPositionSolution();
		++itercontact;
	}
}

void  ChContactContainerNodes::ConstraintsLiFetchSuggestedSpeedSolution()
{
	// Store the last computed reactions into the persistent contact manifold (might
	// be used for warm starting CCP the speed solver):
	std::list<ChContactNode*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsLiFetchSuggestedSpeedSolution();
		++itercontact;
	}
}

void  ChContactContainerNodes::ConstraintsLiFetchSuggestedPositionSolution()
{
	// Store the last computed 'positional' reactions into the persistent contact manifold (might
	// be used for warm starting the CCP position stabilization solver):
	std::list<ChContactNode*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsLiFetchSuggestedPositionSolution();
		++itercontact;
	}
}




} // END_OF_NAMESPACE____


