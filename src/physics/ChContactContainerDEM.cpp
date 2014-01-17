//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChContactContainerDEM.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
#include "physics/ChContactContainerDEM.h"
#include "physics/ChSystem.h"
#include "physics/ChIndexedNodes.h"
#include "physics/ChBodyDEM.h"
#include "collision/ChCModelBulletDEM.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


using namespace collision;
using namespace geometry;





// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChContactContainerDEM> a_registration_ChContactContainerDEM;
 

ChContactContainerDEM::ChContactContainerDEM ()
{ 
	contactlist.clear();
	n_added = 0;
}


ChContactContainerDEM::~ChContactContainerDEM ()
{
	std::list<ChContactDEM*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		delete (*itercontact);
		(*itercontact) = 0;
		++itercontact;
		//contactlist.erase(itercontact); //no! do later with clear(), all together
	}
	contactlist.clear();
}



void ChContactContainerDEM::Update (double mytime)
{
	/*
	ChContactDEM* cntct;
	ChModelBulletDEM* modelA;;
	ChModelBulletDEM* modelB;;
	ChBodyDEM* bodyA;
	ChBodyDEM* bodyB;
	ChVector<> contactForce;
	ChVector<> absForceA, absForceB;
	ChVector<> absTorqueA, absTorqueB;
	ChVector<> pA, pB;
	bool apply_force;
	std::list<ChContactDEM*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		cntct = (ChContactDEM*)(*itercontact);
		modelA = (ChModelBulletDEM*)(cntct->GetModelA());
		modelB = (ChModelBulletDEM*)(cntct->GetModelB());
		bodyA = modelA->GetBody();
		bodyB = modelB->GetBody();

		apply_force=true;
		if (ChSystemMPI* syss = dynamic_cast<ChSystemMPI*>(this->GetSystem()))
		{
			if( ~(*syss).nodeMPI.IsInto( (cntct->GetContactP1() + cntct->GetContactP2())/2.0 ) )
			{
				apply_force=false;
			}
		}
		if (apply_force)
		{
			contactForce = cntct->GetContactForce();

			bodyA->AccumulateForce(contactForce);
			bodyB->AccumulateForce(-contactForce);
			
			pA = cntct->GetContactP1();
			pB = cntct->GetContactP2();
			bodyA->To_abs_forcetorque(contactForce, pA, 0, absForceA, absTorqueA);
			bodyB->To_abs_forcetorque(-contactForce, pB, 0, absForceB, absTorqueB);

			bodyA->AccumulateTorque(absTorqueA);
			bodyB->AccumulateTorque(absTorqueB);

		}

		++itercontact;
	}
	*/
    // Inherit time changes of parent class, basically doing nothing :)
    ChContactContainerBase::Update(mytime);

}

void ChContactContainerDEM::ConstraintsFbLoadForces(double factor)
{
	ChContactDEM* cntct;

	std::list<ChContactDEM*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		cntct = (ChContactDEM*)(*itercontact);
		if ((cntct->GetContactDistance()<0) )
		{
			(*itercontact)->ConstraintsFbLoadForces(factor);
		}
		++itercontact;
	}
}

void ChContactContainerDEM::RemoveAllContacts()
{
	std::list<ChContactDEM*>::iterator itercontact = contactlist.begin();
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


void ChContactContainerDEM::BeginAddContact()
{
	lastcontact = contactlist.begin();
	n_added = 0;
}

void ChContactContainerDEM::EndAddContact()
{
	// remove contacts that are beyond last contact
	while (lastcontact != contactlist.end())
	{
		delete (*lastcontact);
		lastcontact = contactlist.erase(lastcontact);
	}
}


void ChContactContainerDEM::AddContact(const collision::ChCollisionInfo& mcontact)
{
	// Do nothing if the shapes are separated
	if (mcontact.distance >= 0)
		return;

	// Get the contact models.
	ChModelBulletDEM* mmboA = dynamic_cast<ChModelBulletDEM*>(mcontact.modelA);
	ChModelBulletDEM* mmboB = dynamic_cast<ChModelBulletDEM*>(mcontact.modelB);
	if (!mmboA || !mmboB)
		return;

	// Get the associated body frames.
	ChFrame<>* frameA = mmboA->GetBody();
	ChFrame<>* frameB = mmboB->GetBody();
	if (!frameA || !frameB)
		return;

	// Return now if both bodies are inactive
	if (!mmboA->GetBody()->IsActive() && !mmboB->GetBody()->IsActive())
		return;

	ChLcpVariablesBody* varA = &mmboA->GetBody()->Variables();
	ChLcpVariablesBody* varB = &mmboB->GetBody()->Variables();

	// Calculate composite material properties
	const ChSharedPtr<ChMaterialSurfaceDEM>& matsurfA = mmboA->GetBody()->GetMaterialSurfaceDEM();
	const ChSharedPtr<ChMaterialSurfaceDEM>& matsurfB = mmboB->GetBody()->GetMaterialSurfaceDEM();
	double kn_eff, gn_eff, kt_eff, mu_eff;
	ChMaterialSurfaceDEM::compositeMaterial(matsurfA, matsurfB, kn_eff, gn_eff, kt_eff, mu_eff);


	// Create a new contact or reuse an existing one
	if (lastcontact != contactlist.end()) {
		// reuse old contacts
		(*lastcontact)->Reset(mcontact.modelA, mcontact.modelB,
		                      varA, varB,
		                      frameA, frameB,
		                      mcontact.vpA, mcontact.vpB, 
		                      mcontact.vN,
		                      mcontact.distance, 
		                      mcontact.reaction_cache,
		                      kn_eff, gn_eff, kt_eff, mu_eff);
		lastcontact++;
	} else {
		// add new contact
		ChContactDEM* mc = new ChContactDEM(mcontact.modelA, mcontact.modelB,
		                                    varA, varB,
		                                    frameA, frameB,
		                                    mcontact.vpA, mcontact.vpB,
		                                    mcontact.vN,
		                                    mcontact.distance,
		                                    mcontact.reaction_cache,
		                                    kn_eff, gn_eff, kt_eff, mu_eff);
		contactlist.push_back(mc);
		lastcontact = contactlist.end();
	}

	n_added++;
}



void ChContactContainerDEM::ReportAllContacts(ChReportContactCallback* mcallback)
{
	std::list<ChContactDEM*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		bool proceed = mcallback->ReportContactCallback(
					(*itercontact)->GetContactP1(),
					(*itercontact)->GetContactP2(),
					*(*itercontact)->GetContactPlane(),
					(*itercontact)->GetContactDistance(),
					0.0,
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


} // END_OF_NAMESPACE____


