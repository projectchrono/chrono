//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChProximityContainerMeshless.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
#include "physics/ChProximityContainerSPH.h"
#include "physics/ChMatterSPH.h"
#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "collision/ChCModelBulletNode.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


using namespace collision;
using namespace geometry;





// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChProximityContainerSPH> a_registration_ChProximityContainerSPH;
 

ChProximityContainerSPH::ChProximityContainerSPH ()
{ 
	proximitylist.clear();
	n_added = 0;
}


ChProximityContainerSPH::~ChProximityContainerSPH ()
{
	std::list<ChProximitySPH*>::iterator iterproximity = proximitylist.begin();
	while(iterproximity != proximitylist.end())
	{
		delete (*iterproximity);
		(*iterproximity) = 0;
		++iterproximity;
		//proximitylist.erase(iterproximity); //no! do later with clear(), all together
	}
	proximitylist.clear();

	lastproximity = proximitylist.begin();
	n_added = 0;
}



void ChProximityContainerSPH::RemoveAllProximities()
{
	std::list<ChProximitySPH*>::iterator iterproximity = proximitylist.begin();
	while(iterproximity != proximitylist.end())
	{
		delete (*iterproximity);
		(*iterproximity) = 0;
		++iterproximity;
		//proximitylist.erase(iterproximity); //no! do later with clear(), all together
	}
	proximitylist.clear();

	lastproximity = proximitylist.begin();
	n_added = 0;
}


void ChProximityContainerSPH::BeginAddProximities()
{
	lastproximity = proximitylist.begin();
	n_added = 0;
}

void ChProximityContainerSPH::EndAddProximities()
{
	// remove proximities that are beyond last proximity
	while (lastproximity != proximitylist.end())
	{
		delete (*lastproximity);
		lastproximity = proximitylist.erase(lastproximity);
	}
}


void ChProximityContainerSPH::AddProximity(
							  collision::ChCollisionModel* modA, 
							  collision::ChCollisionModel* modB  
							  )
{
	// Fetch the frames of that proximity and other infos

	bool fixedA = false;
	bool fixedB = false;

	ChModelBulletNode* mmpaA = dynamic_cast<ChModelBulletNode*>(modA);
	
	ChModelBulletNode* mmpaB = dynamic_cast<ChModelBulletNode*>(modB);

	if (!(mmpaA && mmpaB))
		return;

	if ((fixedA && fixedB))
		return;

	// Launch the proximity callback, if implemented by the user

	if (this->add_proximity_callback)
	{
		this->add_proximity_callback->ProximityCallback(*modA, *modB);
	}

	// %%%%%%% Create and add a ChProximitySPH object 

	if (lastproximity != proximitylist.end())
	{
		// reuse old proximity pairs
		(*lastproximity)->Reset(mmpaA, mmpaB);
						
		lastproximity++;
	}
	else
	{
		// add new proximity
		ChProximitySPH* mp = new ChProximitySPH( mmpaA, mmpaB);
		
		proximitylist.push_back(mp);
		lastproximity = proximitylist.end();
	}
	n_added++;

	
}



void ChProximityContainerSPH::ReportAllProximities(ChReportProximityCallback* mcallback)
{
	std::list<ChProximitySPH*>::iterator iterproximity = proximitylist.begin();
	while(iterproximity != proximitylist.end())
	{
		bool proceed = mcallback->ReportProximityCallback(
					(*iterproximity)->GetModelA(), 
					(*iterproximity)->GetModelB()  
					);
		if (!proceed) 
			break;
		++iterproximity;
	}
}


////////// LCP INTERFACES ////

static double W_poly6(double r, double h)
{
	if (r < h)
	{
		return (315.0 / (64.0 * CH_C_PI * pow(h,9)) ) * pow((h*h - r*r) ,3);
	}
	else return 0;
}


static double W_sq_visco(double r, double h)
{
	if (r < h)
	{
		return (45.0 / (CH_C_PI * pow(h,6)) ) * (h - r);
	}
	else return 0;
}

static void W_gr_press(ChVector<>& Wresult, const ChVector<>& r, const double r_length, const double h)
{
	if (r_length < h)
	{
		Wresult = r;
		Wresult *= -(45.0 / (CH_C_PI * pow(h,6)) ) * pow((h - r_length),2.0) ;
	}
	else Wresult = VNULL;
}


void ChProximityContainerSPH::AccumulateStep1()
{
	// Per-edge data computation
	std::list<ChProximitySPH*>::iterator iterproximity = proximitylist.begin();
	while(iterproximity != proximitylist.end())
	{
		ChMatterSPH* mmatA = (ChMatterSPH*)(*iterproximity)->GetModelA()->GetPhysicsItem();
		ChMatterSPH* mmatB = (ChMatterSPH*)(*iterproximity)->GetModelB()->GetPhysicsItem();
		ChSharedPtr<ChNodeSPH> mnodeA ( mmatA->GetNode ( ((ChModelBulletNode*)(*iterproximity)->GetModelA())->GetNodeId()  ).DynamicCastTo<ChNodeSPH>() );
		ChSharedPtr<ChNodeSPH> mnodeB ( mmatB->GetNode ( ((ChModelBulletNode*)(*iterproximity)->GetModelB())->GetNodeId()  ).DynamicCastTo<ChNodeSPH>() );
	
		ChVector<> x_A  = mnodeA->GetPos();
		ChVector<> x_B  = mnodeB->GetPos();

		ChVector<> r_BA = x_B  - x_A ;
		double dist_BA = r_BA.Length();

		double W_k_poly6 =  W_poly6( dist_BA, mnodeA->GetKernelRadius() );

		// increment data of connected nodes

		mnodeA->density += mnodeB->GetMass() * W_k_poly6;
		mnodeB->density += mnodeA->GetMass() * W_k_poly6;

		++iterproximity;
	}

}

void ChProximityContainerSPH::AccumulateStep2()
{
	// Per-edge data computation (transfer stress to forces)
	std::list<ChProximitySPH*>::iterator iterproximity = proximitylist.begin();
	while(iterproximity != proximitylist.end())
	{
		ChMatterSPH* mmatA = (ChMatterSPH*)(*iterproximity)->GetModelA()->GetPhysicsItem();
		ChMatterSPH* mmatB = (ChMatterSPH*)(*iterproximity)->GetModelB()->GetPhysicsItem();
		ChSharedPtr<ChNodeSPH> mnodeA ( mmatA->GetNode ( ((ChModelBulletNode*)(*iterproximity)->GetModelA())->GetNodeId()  ).DynamicCastTo<ChNodeSPH>() );
		ChSharedPtr<ChNodeSPH> mnodeB ( mmatB->GetNode ( ((ChModelBulletNode*)(*iterproximity)->GetModelB())->GetNodeId()  ).DynamicCastTo<ChNodeSPH>() );
	
		ChVector<> x_A  = mnodeA->GetPos();
		ChVector<> x_B  = mnodeB->GetPos();

		ChVector<> r_BA = x_B - x_A;
		double dist_BA = r_BA.Length();

		// increment pressure forces

		ChVector<> W_k_press;
		W_gr_press( W_k_press, r_BA, dist_BA, mnodeA->GetKernelRadius() );

		double avg_press = 0.5*(mnodeA->pressure + mnodeB->pressure);
	
		ChVector<> pressureForceA  =   W_k_press * mnodeA->volume * avg_press * mnodeB->volume;
		mnodeA->UserForce += pressureForceA;

		//ChVector<> pressureForceB  = - W_k_press * mnodeB->volume * avg_dens * mnodeA->volume;
		mnodeB->UserForce -= pressureForceA;


		// increment viscous forces..

		double W_k_visc =  W_sq_visco( dist_BA, mnodeA->GetKernelRadius() );
		ChVector<> velBA =  mnodeB->GetPos_dt() - mnodeA->GetPos_dt();

		double avg_viscosity =  0.5*(mmatA->GetMaterial().Get_viscosity() + mmatB->GetMaterial().Get_viscosity());

		ChVector<> viscforceBA =  velBA * ( mnodeA->volume * avg_viscosity * mnodeB->volume * W_k_visc );
		mnodeA->UserForce += viscforceBA;
		mnodeB->UserForce -= viscforceBA;


		++iterproximity;
	}

}




} // END_OF_NAMESPACE____


