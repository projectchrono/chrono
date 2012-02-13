///////////////////////////////////////////////////
//
//   ChContactContainer.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
#include "physics/ChContactContainer.h"
#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "physics/ChParticlesClones.h"
#include "lcp/ChLcpConstraintTwoContactN.h"
#include "collision/ChCModelBulletBody.h"
#include "collision/ChCModelBulletParticle.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


using namespace collision;
using namespace geometry;





// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChContactContainer> a_registration_ChContactContainer;
 

ChContactContainer::ChContactContainer ()
{ 
	contactlist.clear();
	n_added = 0;

	contactlist_roll.clear();
	n_added_roll = 0;
}


ChContactContainer::~ChContactContainer ()
{
	std::list<ChContact*>::iterator itercontact = contactlist.begin();
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


	std::list<ChContactRolling*>::iterator itercontact_roll = contactlist_roll.begin();
	while(itercontact_roll != contactlist_roll.end())
	{
		delete (*itercontact_roll);
		(*itercontact_roll) = 0;
		++itercontact_roll;
	}
	contactlist_roll.clear();

	lastcontact_roll = contactlist_roll.begin();
	n_added_roll = 0;
}



void ChContactContainer::Update (double mytime)
{
    // Inherit time changes of parent class, basically doing nothing :)
    ChContactContainerBase::Update(mytime);

}


void ChContactContainer::RemoveAllContacts()
{
	std::list<ChContact*>::iterator itercontact = contactlist.begin();
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

	std::list<ChContactRolling*>::iterator itercontact_roll = contactlist_roll.begin();
	while(itercontact_roll != contactlist_roll.end())
	{
		delete (*itercontact_roll);
		(*itercontact_roll) = 0;
		++itercontact_roll;
	}
	contactlist_roll.clear();

	lastcontact_roll = contactlist_roll.begin();
	n_added_roll = 0;
}


void ChContactContainer::BeginAddContact()
{
	lastcontact = contactlist.begin();
	n_added = 0;

	lastcontact_roll = contactlist_roll.begin();
	n_added_roll = 0;
}

void ChContactContainer::EndAddContact()
{
	// remove contacts that are beyond last contact
	while (lastcontact != contactlist.end())
	{
		delete (*lastcontact);
		lastcontact = contactlist.erase(lastcontact);
	}
	while (lastcontact_roll != contactlist_roll.end())
	{
		delete (*lastcontact_roll);
		lastcontact_roll = contactlist_roll.erase(lastcontact_roll);
	}
}


void ChContactContainer::AddContact(const collision::ChCollisionInfo& mcontact)
{
	// Fetch the frames of that contact and other infos

	ChFrame<>* frameA =0;
	ChFrame<>* frameB =0;
	bool inactiveA = false;
	bool inactiveB = false;
	ChLcpVariablesBody* varA = 0;
	ChLcpVariablesBody* varB = 0;
	ChSharedPtr<ChMaterialSurface> mmatA;
	ChSharedPtr<ChMaterialSurface> mmatB;

	if (ChModelBulletBody* mmboA = dynamic_cast<ChModelBulletBody*>(mcontact.modelA))
	{
		frameA = mmboA->GetBody();
		varA    =&mmboA->GetBody()->Variables();
		inactiveA = !mmboA->GetBody()->IsActive();
		mmatA = mmboA->GetBody()->GetMaterialSurface();
	}
	if (ChModelBulletParticle* mmpaA = dynamic_cast<ChModelBulletParticle*>(mcontact.modelA))
	{
		frameA = &(mmpaA->GetParticles()->GetParticle(mmpaA->GetParticleId()));
		varA   = (ChLcpVariablesBody*) &(mmpaA->GetParticles()->GetParticle(mmpaA->GetParticleId())).Variables();
		if (ChParticlesClones* mpclone = dynamic_cast<ChParticlesClones*>(mmpaA->GetParticles()))
		{
			mmatA = mpclone->GetMaterialSurface();
		}
	}

	if (ChModelBulletBody* mmboB = dynamic_cast<ChModelBulletBody*>(mcontact.modelB))
	{
		frameB = mmboB->GetBody();
		varB    =&mmboB->GetBody()->Variables();
		inactiveB = !mmboB->GetBody()->IsActive();
		mmatB = mmboB->GetBody()->GetMaterialSurface();
	}
	if (ChModelBulletParticle* mmpaB = dynamic_cast<ChModelBulletParticle*>(mcontact.modelB))
	{
		frameB = &(mmpaB->GetParticles()->GetParticle(mmpaB->GetParticleId()));
		varB   = (ChLcpVariablesBody*) &(mmpaB->GetParticles()->GetParticle(mmpaB->GetParticleId())).Variables();
		if (ChParticlesClones* mpclone = dynamic_cast<ChParticlesClones*>(mmpaB->GetParticles()))
		{
			mmatB = mpclone->GetMaterialSurface();
		}
	}

	if (!(frameA && frameB))
		return;

	assert (varA);
	assert (varB);

	if ((inactiveA && inactiveB))
		return;

	// Compute default material-couple values.

	ChMaterialCouple mat;

	mat.static_friction		= (float)ChMin( mmatA->static_friction,		mmatB->static_friction);
	mat.rolling_friction	= (float)ChMin( mmatA->rolling_friction,	mmatB->rolling_friction);
	mat.spinning_friction	= (float)ChMin( mmatA->spinning_friction,	mmatB->spinning_friction);
	mat.restitution			= (float)ChMin( mmatA->restitution,			mmatB->restitution);
	mat.cohesion			= (float)ChMin( mmatA->cohesion,			mmatB->cohesion);
	mat.dampingf			= (float)ChMin( mmatA->dampingf,			mmatB->dampingf);
	mat.compliance			= (float)(mmatA->compliance+mmatB->compliance);// (float)ChMin( mmatA->compliance,			mmatB->compliance);
	mat.complianceT			= (float)(mmatA->complianceT+mmatB->complianceT); // (float)ChMin( mmatA->complianceT,			mmatB->complianceT);
	

	// Launch the contact callback, if any, to set custom friction & material 
	// properties, if implemented by the user:

	if (this->add_contact_callback)
	{
		this->add_contact_callback->ContactCallback(mcontact, mat);
	}

	// %%%%%%% Create and add a ChContact object (or ChContactRolling if there is spinn. or roll.friction) %%%%%%%

	if ((mat.rolling_friction == 0) && (mat.spinning_friction == 0))
	{
		if (lastcontact != contactlist.end())
		{
			// reuse old contacts
			(*lastcontact)->Reset(		  mcontact.modelA,
										  mcontact.modelB,
										  varA, varB,
										  frameA, frameB,
										  mcontact.vpA, 
										  mcontact.vpB, 
										  mcontact.vN,
										  mcontact.distance, 
										  mcontact.reaction_cache,
										  mat);
			lastcontact++;
		}
		else
		{
			// add new contact
			ChContact* mc = new ChContact(mcontact.modelA,
										  mcontact.modelB,
										  varA, varB,
										  frameA, frameB,
										  mcontact.vpA, 
										  mcontact.vpB, 
										  mcontact.vN,
										  mcontact.distance, 
										  mcontact.reaction_cache,
										  mat);
			contactlist.push_back(mc);
			lastcontact = contactlist.end();
		}
		n_added++;
	}
	else
	{
		if (lastcontact_roll != contactlist_roll.end())
		{
			// reuse old rolling contacts
			(*lastcontact_roll)->Reset(	  mcontact.modelA,
										  mcontact.modelB,
										  varA, varB,
										  frameA, frameB,
										  mcontact.vpA, 
										  mcontact.vpB, 
										  mcontact.vN,
										  mcontact.distance, 
										  mcontact.reaction_cache,
										  mat);
			lastcontact_roll++;
		}
		else
		{
			// add new contact
			ChContactRolling* mc = new ChContactRolling(mcontact.modelA,
										  mcontact.modelB,
										  varA, varB,
										  frameA, frameB,
										  mcontact.vpA, 
										  mcontact.vpB, 
										  mcontact.vN,
										  mcontact.distance, 
										  mcontact.reaction_cache,
										  mat);
			contactlist_roll.push_back(mc);
			lastcontact_roll = contactlist_roll.end();
		}
		n_added_roll ++;
	}

	
}



void ChContactContainer::ReportAllContacts(ChReportContactCallback* mcallback)
{
	std::list<ChContact*>::iterator itercontact = contactlist.begin();
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

	std::list<ChContactRolling*>::iterator itercontact_roll = contactlist_roll.begin();
	while(itercontact_roll != contactlist_roll.end())
	{
		bool proceed = mcallback->ReportContactCallback(
					(*itercontact_roll)->GetContactP1(),
					(*itercontact_roll)->GetContactP2(),
					*(*itercontact_roll)->GetContactPlane(),
					(*itercontact_roll)->GetContactDistance(),
					(*itercontact_roll)->GetFriction(),
					(*itercontact_roll)->GetContactForce(),
					(*itercontact_roll)->GetContactTorque(),
					(*itercontact_roll)->GetModelA(), 
					(*itercontact_roll)->GetModelB()  
					);
		if (!proceed) 
			break;
		++itercontact_roll;
	}
}


////////// LCP INTERFACES ////


void ChContactContainer::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	std::list<ChContact*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->InjectConstraints(mdescriptor);
		++itercontact;
	}
	std::list<ChContactRolling*>::iterator itercontact_roll = contactlist_roll.begin();
	while(itercontact_roll != contactlist_roll.end())
	{
		(*itercontact_roll)->InjectConstraints(mdescriptor);
		++itercontact_roll;
	}
}

void ChContactContainer::ConstraintsBiReset()
{
	std::list<ChContact*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsBiReset();
		++itercontact;
	}
	std::list<ChContactRolling*>::iterator itercontact_roll = contactlist_roll.begin();
	while(itercontact_roll != contactlist_roll.end())
	{
		(*itercontact_roll)->ConstraintsBiReset();
		++itercontact_roll;
	}
}
 
void ChContactContainer::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	std::list<ChContact*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
		++itercontact;
	}
	std::list<ChContactRolling*>::iterator itercontact_roll = contactlist_roll.begin();
	while(itercontact_roll != contactlist_roll.end())
	{
		(*itercontact_roll)->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
		++itercontact_roll;
	}
}


void ChContactContainer::ConstraintsLoadJacobians()
{
	// already loaded when ChContact objects are created
}
 

void ChContactContainer::ConstraintsFetch_react(double factor)
{
	// From constraints to react vector:
	std::list<ChContact*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsFetch_react(factor);
		++itercontact;
	}
	std::list<ChContactRolling*>::iterator itercontact_roll = contactlist_roll.begin();
	while(itercontact_roll != contactlist_roll.end())
	{
		(*itercontact_roll)->ConstraintsFetch_react(factor);
		++itercontact_roll;
	}
}


// Following functions are for exploiting the contact persistence


void  ChContactContainer::ConstraintsLiLoadSuggestedSpeedSolution()
{
	// Fetch the last computed impulsive reactions from the persistent contact manifold (could
	// be used for warm starting the CCP speed solver):
	std::list<ChContact*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsLiLoadSuggestedSpeedSolution();
		++itercontact;
	}
	std::list<ChContactRolling*>::iterator itercontact_roll = contactlist_roll.begin();
	while(itercontact_roll != contactlist_roll.end())
	{
		(*itercontact_roll)->ConstraintsLiLoadSuggestedSpeedSolution();
		++itercontact_roll;
	}
}

void  ChContactContainer::ConstraintsLiLoadSuggestedPositionSolution()
{
	// Fetch the last computed 'positional' reactions from the persistent contact manifold (could
	// be used for warm starting the CCP position stabilization solver):
	std::list<ChContact*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsLiLoadSuggestedPositionSolution();
		++itercontact;
	}
	std::list<ChContactRolling*>::iterator itercontact_roll = contactlist_roll.begin();
	while(itercontact_roll != contactlist_roll.end())
	{
		(*itercontact_roll)->ConstraintsLiLoadSuggestedPositionSolution();
		++itercontact_roll;
	}
}

void  ChContactContainer::ConstraintsLiFetchSuggestedSpeedSolution()
{
	// Store the last computed reactions into the persistent contact manifold (might
	// be used for warm starting CCP the speed solver):
	std::list<ChContact*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsLiFetchSuggestedSpeedSolution();
		++itercontact;
	}
	std::list<ChContactRolling*>::iterator itercontact_roll = contactlist_roll.begin();
	while(itercontact_roll != contactlist_roll.end())
	{
		(*itercontact_roll)->ConstraintsLiFetchSuggestedSpeedSolution();
		++itercontact_roll;
	}
}

void  ChContactContainer::ConstraintsLiFetchSuggestedPositionSolution()
{
	// Store the last computed 'positional' reactions into the persistent contact manifold (might
	// be used for warm starting the CCP position stabilization solver):
	std::list<ChContact*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsLiFetchSuggestedPositionSolution();
		++itercontact;
	}
	std::list<ChContactRolling*>::iterator itercontact_roll = contactlist_roll.begin();
	while(itercontact_roll != contactlist_roll.end())
	{
		(*itercontact_roll)->ConstraintsLiFetchSuggestedPositionSolution();
		++itercontact_roll;
	}
}




} // END_OF_NAMESPACE____


