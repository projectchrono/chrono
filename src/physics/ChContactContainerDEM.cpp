///////////////////////////////////////////////////
//
//   ChContactContainerDEM.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
#include "physics/ChContactContainerDEM.h"
#include "physics/ChSystem.h"
#include "unit_MPI/ChSystemMPI.h"
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
			/*
			bodyA->AccumulateTorque(absTorqueA);
			bodyB->AccumulateTorque(absTorqueB);
			*/
		}

		++itercontact;
	}
    // Inherit time changes of parent class, basically doing nothing :)
    ChContactContainerBase::Update(mytime);

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
	// Fetch the frames of that contact and other infos
	ChFrame<>* frameA =0;
	ChFrame<>* frameB =0;
	bool inactiveA = false;
	bool inactiveB = false;
	ChLcpVariablesBody* varA = 0;
	ChLcpVariablesBody* varB = 0;
	double springA=0;
	double springB=0;
	double dampingA=0;
	double dampingB=0;

	if (ChModelBulletDEM* mmboA = dynamic_cast<ChModelBulletDEM*>(mcontact.modelA))
	{
		frameA = mmboA->GetBody();
		varA    =&mmboA->GetBody()->Variables();
		springA = mmboA->GetBody()->GetSpringCoefficient();
		dampingA= mmboA->GetBody()->GetDampingCoefficient();
		inactiveA = !mmboA->GetBody()->IsActive();
	}
	if (ChModelBulletDEM* mmboB = dynamic_cast<ChModelBulletDEM*>(mcontact.modelB))
	{
		frameB = mmboB->GetBody();
		varB    =&mmboB->GetBody()->Variables();
		springB = mmboB->GetBody()->GetSpringCoefficient();
		dampingB= mmboB->GetBody()->GetDampingCoefficient();
		inactiveB = !mmboB->GetBody()->IsActive();
	}

	if (!(frameA && frameB))
		return;

	assert (varA);
	assert (varB);

	if ((inactiveA && inactiveB))
		return;

	// compute the contact spring and damping coefficients
	double kn_eff = 1/((1/springA)+(1/springB));
	double gn_eff = 1/((1/dampingA)+(1/dampingB));

	// %%%%%%% Create and add a ChContact object  %%%%%%%

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
									  kn_eff,
									  gn_eff);
		lastcontact++;
	}
	else
	{
		// add new contact
		ChContactDEM* mc = new ChContactDEM(mcontact.modelA,
										  mcontact.modelB,
										  varA, varB,
										  frameA, frameB,
										  mcontact.vpA, 
										  mcontact.vpB, 
										  mcontact.vN,
										  mcontact.distance, 
										  mcontact.reaction_cache,
										  kn_eff,
										  gn_eff);
		contactlist.push_back(mc);
		lastcontact = contactlist.end();
	}
	ChModelBulletDEM* mmboA = dynamic_cast<ChModelBulletDEM*>(mcontact.modelA);
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


