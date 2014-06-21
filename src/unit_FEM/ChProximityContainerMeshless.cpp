//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File authors: Alessandro Tasora

  
#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "collision/ChCModelBulletNode.h"
#include "ChMatterMeshless.h"
#include "ChProximityContainerMeshless.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{

using namespace fem;
using namespace collision;
using namespace geometry;





// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChProximityContainerMeshless> a_registration_ChProximityContainerMeshless;
 

ChProximityContainerMeshless::ChProximityContainerMeshless ()
{ 
	proximitylist.clear();
	n_added = 0;
}


ChProximityContainerMeshless::~ChProximityContainerMeshless ()
{
	std::list<ChProximityMeshless*>::iterator iterproximity = proximitylist.begin();
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




void ChProximityContainerMeshless::RemoveAllProximities()
{
	std::list<ChProximityMeshless*>::iterator iterproximity = proximitylist.begin();
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


void ChProximityContainerMeshless::BeginAddProximities()
{
	lastproximity = proximitylist.begin();
	n_added = 0;
}

void ChProximityContainerMeshless::EndAddProximities()
{
	// remove proximities that are beyond last proximity
	while (lastproximity != proximitylist.end())
	{
		delete (*lastproximity);
		lastproximity = proximitylist.erase(lastproximity);
	}
}


void ChProximityContainerMeshless::AddProximity(
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

	// %%%%%%% Create and add a ChProximityMeshless object 

	if (lastproximity != proximitylist.end())
	{
		// reuse old proximity pairs
		(*lastproximity)->Reset(mmpaA, mmpaB);
						
		lastproximity++;
	}
	else
	{
		// add new proximity
		ChProximityMeshless* mp = new ChProximityMeshless( mmpaA, mmpaB);
		
		proximitylist.push_back(mp);
		lastproximity = proximitylist.end();
	}
	n_added++;

	
}



void ChProximityContainerMeshless::ReportAllProximities(ChReportProximityCallback* mcallback)
{
	std::list<ChProximityMeshless*>::iterator iterproximity = proximitylist.begin();
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

static double W_sph(double r, double h)
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



void ChProximityContainerMeshless::AccumulateStep1()
{
	// Per-edge data computation
	std::list<ChProximityMeshless*>::iterator iterproximity = proximitylist.begin();
	while(iterproximity != proximitylist.end())
	{
		ChMatterMeshless* mmatA = (ChMatterMeshless*)(*iterproximity)->GetModelA()->GetPhysicsItem();
		ChMatterMeshless* mmatB = (ChMatterMeshless*)(*iterproximity)->GetModelB()->GetPhysicsItem();
		ChSharedPtr<ChNodeMeshless> mnodeA = mmatA->GetNode ( ((ChModelBulletNode*)(*iterproximity)->GetModelA())->GetNodeId()  ).DynamicCastTo<ChNodeMeshless>();
		ChSharedPtr<ChNodeMeshless> mnodeB = mmatB->GetNode ( ((ChModelBulletNode*)(*iterproximity)->GetModelB())->GetNodeId()  ).DynamicCastTo<ChNodeMeshless>();
	
		ChVector<> x_A  = mnodeA->GetPos();
		ChVector<> x_B  = mnodeB->GetPos();
		ChVector<> x_Aref  = mnodeA->GetPosReference();
		ChVector<> x_Bref  = mnodeB->GetPosReference();
		ChVector<> u_A = (x_A -x_Aref);
		ChVector<> u_B = (x_B -x_Bref);

		ChVector<> d_BA = x_Bref - x_Aref;
		ChVector<> g_BA = u_B - u_A;
		double dist_BA = d_BA.Length();
		double W_BA =  W_sph( dist_BA, mnodeA->GetKernelRadius() );
		double W_AB =  W_sph( dist_BA, mnodeB->GetKernelRadius() );

		// increment data of connected nodes

		mnodeA->density += mnodeB->GetMass() * W_BA;
		mnodeB->density += mnodeA->GetMass() * W_AB;

		ChMatrixNM<double, 3,1> mdist;
		mdist.PasteVector(d_BA,0,0);
		ChMatrixNM<double, 3,1> mdistT;
		mdistT.PasteVector(d_BA,0,0);
		ChMatrix33<> ddBA; 
		ddBA.MatrMultiplyT(mdist, mdistT);
		ChMatrix33<> ddAB(ddBA);

		ddBA.MatrScale(W_BA);
		mnodeA->Amoment.MatrInc(ddBA);	// increment the moment matrix: Aa += d_BA*d_BA'*W_BA
			
		ddAB.MatrScale(W_AB);
		mnodeB->Amoment.MatrInc(ddAB);	// increment the moment matrix: Ab += d_AB*d_AB'*W_AB


		ChVector<> m_inc_BA = (d_BA)  * W_BA;

		ChVector<> m_inc_AB = (-d_BA) * W_AB;

		ChVector<> dwg;					// increment the J matrix
		dwg = m_inc_BA * g_BA.x;
		mnodeA->J.PasteSumVector(dwg,0,0);
		dwg = m_inc_BA * g_BA.y;
		mnodeA->J.PasteSumVector(dwg,0,1);
		dwg = m_inc_BA * g_BA.z;
		mnodeA->J.PasteSumVector(dwg,0,2);		

		dwg = m_inc_AB * (-g_BA.x);		// increment the J matrix
		mnodeB->J.PasteSumVector(dwg,0,0);
		dwg = m_inc_AB * (-g_BA.y);
		mnodeB->J.PasteSumVector(dwg,0,1);
		dwg = m_inc_AB * (-g_BA.z);
		mnodeB->J.PasteSumVector(dwg,0,2);


		++iterproximity;
	}

}

void ChProximityContainerMeshless::AccumulateStep2()
{
	// Per-edge data computation (transfer stress to forces)
	std::list<ChProximityMeshless*>::iterator iterproximity = proximitylist.begin();
	while(iterproximity != proximitylist.end())
	{
		ChMatterMeshless* mmatA = (ChMatterMeshless*)(*iterproximity)->GetModelA()->GetPhysicsItem();
		ChMatterMeshless* mmatB = (ChMatterMeshless*)(*iterproximity)->GetModelB()->GetPhysicsItem();
		ChSharedPtr<ChNodeMeshless> mnodeA = mmatA->GetNode ( ((ChModelBulletNode*)(*iterproximity)->GetModelA())->GetNodeId()  ).DynamicCastTo<ChNodeMeshless>();
		ChSharedPtr<ChNodeMeshless> mnodeB = mmatB->GetNode ( ((ChModelBulletNode*)(*iterproximity)->GetModelB())->GetNodeId()  ).DynamicCastTo<ChNodeMeshless>();
	
		ChVector<> x_A  = mnodeA->GetPos();
		ChVector<> x_B  = mnodeB->GetPos();
		ChVector<> x_Aref  = mnodeA->GetPosReference();
		ChVector<> x_Bref  = mnodeB->GetPosReference();
		ChVector<> u_A = (x_A -x_Aref);
		ChVector<> u_B = (x_B -x_Bref);

		ChVector<> d_BA = x_Bref - x_Aref;

		double dist_BA = d_BA.Length();
		double W_BA =  W_sph( dist_BA, mnodeA->GetKernelRadius() );
		double W_AB =  W_sph( dist_BA, mnodeB->GetKernelRadius() );

		// increment elastoplastic forces of connected nodes
	
		mnodeA->UserForce += mnodeA->FA * (d_BA*W_BA);
		mnodeB->UserForce -= mnodeA->FA * (d_BA*W_BA);

		mnodeB->UserForce += mnodeB->FA * (d_BA*(-W_AB));
		mnodeA->UserForce -= mnodeB->FA * (d_BA*(-W_AB));

		// increment viscous forces..

		ChVector<> r_BA  = x_B - x_A;
		double r_length = r_BA.Length();
		double W_BA_visc =  W_sq_visco( r_length, mnodeA->GetKernelRadius() );
		double W_AB_visc =  W_sq_visco( r_length, mnodeB->GetKernelRadius() );
		ChVector<> velBA =  mnodeB->GetPos_dt() - mnodeA->GetPos_dt();

		double avg_viscosity = 0.5*(mmatA->GetViscosity()+mmatB->GetViscosity());

		ChVector<> viscforceBA = velBA * ( mnodeA->volume * avg_viscosity * mnodeB->volume * W_BA_visc );
		mnodeA->UserForce += viscforceBA;
		mnodeB->UserForce -= viscforceBA;

		++iterproximity;
	}

}




} // END_OF_NAMESPACE____


