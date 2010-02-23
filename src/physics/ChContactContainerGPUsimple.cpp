#ifndef CH_NOCUDA 

///////////////////////////////////////////////////
//
//   ChContactContainerGPUsimple.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
#include "physics/ChContactContainerGPUsimple.h"
#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "physics/ChParticles.h"
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
ChClassRegister<ChContactContainerGPUsimple> a_registration_ChContactContainerGPUsimple;
 

ChContactContainerGPUsimple::ChContactContainerGPUsimple ()
{ 
	contactlist.clear();
	n_added = 0;
}


ChContactContainerGPUsimple::~ChContactContainerGPUsimple ()
{
	std::list<ChContactGPUsimple*>::iterator itercontact = contactlist.begin();
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



void ChContactContainerGPUsimple::Update (double mytime)
{
    // Inherit time changes of parent class, basically doing nothing :)
    ChContactContainerBase::Update(mytime);

}


void ChContactContainerGPUsimple::RemoveAllContacts()
{
	std::list<ChContactGPUsimple*>::iterator itercontact = contactlist.begin();
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


void ChContactContainerGPUsimple::BeginAddContact()
{
	lastcontact = contactlist.begin();
	n_added = 0;
}

void ChContactContainerGPUsimple::EndAddContact()
{
	// remove contacts that are beyond last contact
	while (lastcontact != contactlist.end())
	{
		delete (*lastcontact);
		lastcontact = contactlist.erase(lastcontact);
	}
}


void ChContactContainerGPUsimple::AddContact(const collision::ChCollisionInfo& mcontact)
{
	// Fetch the frames of that contact and other infos

	ChFrame<>* frameA =0;
	ChFrame<>* frameB =0;
	bool fixedA = false;
	bool fixedB = false;
	float frictionA, frictionB;
	ChLcpVariablesBody* varA = 0;
	ChLcpVariablesBody* varB = 0;

	if (ChModelBulletBody* mmboA = dynamic_cast<ChModelBulletBody*>(mcontact.modelA))
	{
		frameA = mmboA->GetBody();
		varA    =&mmboA->GetBody()->Variables();
		fixedA  = mmboA->GetBody()->GetBodyFixed();
		frictionA = mmboA->GetBody()->GetSfriction();		
	}
	if (ChModelBulletParticle* mmpaA = dynamic_cast<ChModelBulletParticle*>(mcontact.modelA))
	{
		frameA = &(mmpaA->GetParticles()->GetParticle(mmpaA->GetParticleId()));
		varA    = &(mmpaA->GetParticles()->GetParticle(mmpaA->GetParticleId())).variables;
		frictionA = mmpaA->GetParticles()->GetSfriction();
	}

	if (ChModelBulletBody* mmboB = dynamic_cast<ChModelBulletBody*>(mcontact.modelB))
	{
		frameB = mmboB->GetBody();
		varB    =&mmboB->GetBody()->Variables();
		fixedB  = mmboB->GetBody()->GetBodyFixed();
		frictionB = mmboB->GetBody()->GetSfriction();
	}
	if (ChModelBulletParticle* mmpaB = dynamic_cast<ChModelBulletParticle*>(mcontact.modelB))
	{
		frameB = &(mmpaB->GetParticles()->GetParticle(mmpaB->GetParticleId()));
		varB    = &(mmpaB->GetParticles()->GetParticle(mmpaB->GetParticleId())).variables;
		frictionB = mmpaB->GetParticles()->GetSfriction();
	}

	assert (frameA);
	assert (frameB);
	assert (varA);
	assert (varB);

	if ((fixedA && fixedB))
		return;

	// Compute default material-couple values

	ChMaterialCouple mat;
	mat.static_friction = (frictionA + frictionB)*0.5f;
	//mat.sliding_friction = ...
	//mat.rolling_friction = ... etc //***TO DO***


	// Launch the contact callback, if any, to set custom friction & material 
	// properties, if implemented by the user:

	if (this->add_contact_callback)
	{
		this->add_contact_callback->ContactCallback(mcontact, mat);
	}

	// %%%%%%% Create and add the ChContactGPUsimple object! %%%%%%%

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
									  mat.static_friction);
		lastcontact++;
	}
	else
	{
		// add new contact
		ChContactGPUsimple* mc = new ChContactGPUsimple(mcontact.modelA,
									  mcontact.modelB,
									  varA, varB,
									  frameA, frameB,
									  mcontact.vpA, 
									  mcontact.vpB, 
									  mcontact.vN,
									  mcontact.distance, 
									  mcontact.reaction_cache,
									  mat.static_friction);
		contactlist.push_back(mc);
		lastcontact = contactlist.end();
	}

	n_added++;
}



void ChContactContainerGPUsimple::ReportAllContacts(ChReportContactCallback* mcallback)
{
	std::list<ChContactGPUsimple*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		ChMatrix33<> cplane;
		ChVector<> Vx, Vy, Vz;
		ChVector<> Vsingul(VECT_Y);
		XdirToDxDyDz(&ChVector<>((*itercontact)->GetContactNormal()), &Vsingul, &Vx,  &Vy, &Vz);
		cplane.Set_A_axis(Vx,Vy,Vz);

		bool proceed = mcallback->ReportContactCallback(
					(*itercontact)->GetContactP1(),
					(*itercontact)->GetContactP2(),
					cplane,
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


void ChContactContainerGPUsimple::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	std::list<ChContactGPUsimple*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->InjectConstraints(mdescriptor);
		++itercontact;
	}
}

void ChContactContainerGPUsimple::ConstraintsBiReset()
{
	std::list<ChContactGPUsimple*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsBiReset();
		++itercontact;
	}
}
 
void ChContactContainerGPUsimple::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	std::list<ChContactGPUsimple*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
		++itercontact;
	}
}


void ChContactContainerGPUsimple::ConstraintsLoadJacobians()
{
	// already loaded when ChContactGPUsimple objects are created
}
 

void ChContactContainerGPUsimple::ConstraintsFetch_react(double factor)
{
	// From constraints to react vector:
	std::list<ChContactGPUsimple*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsFetch_react(factor);
		++itercontact;
	}
}


// Following functions are for exploiting the contact persistence


void  ChContactContainerGPUsimple::ConstraintsLiLoadSuggestedSpeedSolution()
{
	// Fetch the last computed impulsive reactions from the persistent contact manifold (could
	// be used for warm starting the CCP speed solver):
	std::list<ChContactGPUsimple*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsLiLoadSuggestedSpeedSolution();
		++itercontact;
	}
}

void  ChContactContainerGPUsimple::ConstraintsLiLoadSuggestedPositionSolution()
{
	// Fetch the last computed 'positional' reactions from the persistent contact manifold (could
	// be used for warm starting the CCP position stabilization solver):
	std::list<ChContactGPUsimple*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsLiLoadSuggestedPositionSolution();
		++itercontact;
	}
}

void  ChContactContainerGPUsimple::ConstraintsLiFetchSuggestedSpeedSolution()
{
	// Store the last computed reactions into the persistent contact manifold (might
	// be used for warm starting CCP the speed solver):
	std::list<ChContactGPUsimple*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsLiFetchSuggestedSpeedSolution();
		++itercontact;
	}
}

void  ChContactContainerGPUsimple::ConstraintsLiFetchSuggestedPositionSolution()
{
	// Store the last computed 'positional' reactions into the persistent contact manifold (might
	// be used for warm starting the CCP position stabilization solver):
	std::list<ChContactGPUsimple*>::iterator itercontact = contactlist.begin();
	while(itercontact != contactlist.end())
	{
		(*itercontact)->ConstraintsLiFetchSuggestedPositionSolution();
		++itercontact;
	}
}




} // END_OF_NAMESPACE____



#endif  // end of ! CH_NOCUDA
 
