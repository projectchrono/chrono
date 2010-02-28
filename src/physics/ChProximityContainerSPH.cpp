///////////////////////////////////////////////////
//
//   ChProximityContainerSPH.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
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



void ChProximityContainerSPH::Update (double mytime)
{
    // Inherit time changes of parent class, basically doing nothing :)
    ChProximityContainerBase::Update(mytime);

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

double W_sph(double r, double h)
{
	if (r < h)
	{
		return (315.0 / (64.0 * CH_C_PI * pow(h,9)) ) * pow((h*h - r*r) ,3);
	}
	else return 0;
}


double W_sq_visco(double r, double h)
{
	if (r < h)
	{
		return (45.0 / (CH_C_PI * pow(h,6)) ) * (h - r);
	}
	else return 0;
}



void ChProximityContainerSPH::AccumulateStep1()
{
	// Per-edge data computation
	std::list<ChProximitySPH*>::iterator iterproximity = proximitylist.begin();
	while(iterproximity != proximitylist.end())
	{
		ChMatterSPH* mmatA = (ChMatterSPH*)(*iterproximity)->GetModelA()->GetPhysicsItem();
		ChMatterSPH* mmatB = (ChMatterSPH*)(*iterproximity)->GetModelB()->GetPhysicsItem();
		ChNodeSPH* mnodeA =(ChNodeSPH*) &mmatA->GetNode ( ((ChModelBulletNode*)(*iterproximity)->GetModelA())->GetNodeId()  );
		ChNodeSPH* mnodeB =(ChNodeSPH*) &mmatB->GetNode ( ((ChModelBulletNode*)(*iterproximity)->GetModelB())->GetNodeId()  );
	
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
		mnodeA->m_v = mnodeA->m_v - m_inc_BA; // increment the m_v vector

		ChVector<> m_inc_AB = (-d_BA) * W_AB;
		mnodeB->m_v = mnodeB->m_v - m_inc_AB; // increment the m_v vector


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

		// viscous forces..
		double W_BA_visc =  W_sq_visco( dist_BA, mnodeA->GetKernelRadius() );
		double W_AB_visc =  W_sq_visco( dist_BA, mnodeB->GetKernelRadius() );
		ChVector<> velBA =  mnodeB->GetPos_dt() - mnodeA->GetPos_dt();
		ChVector<> viscforceBA =  velBA *   ( 0.5*(mmatA->GetViscosity()+mmatB->GetViscosity()) * mnodeB->volume * W_BA_visc );
		mnodeA->UserForce += viscforceBA;
		ChVector<> viscforceAB = -velBA *   ( 0.5*(mmatA->GetViscosity()+mmatB->GetViscosity()) * mnodeA->volume * W_AB_visc );
		mnodeB->UserForce += viscforceAB;

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
		ChNodeSPH* mnodeA =(ChNodeSPH*) &mmatA->GetNode ( ((ChModelBulletNode*)(*iterproximity)->GetModelA())->GetNodeId()  );
		ChNodeSPH* mnodeB =(ChNodeSPH*) &mmatB->GetNode ( ((ChModelBulletNode*)(*iterproximity)->GetModelB())->GetNodeId()  );
	
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

		// increment forces of connected nodes

		ChMatrix33<> mtensor;

		mnodeA->t_stress.ConvertToMatrix(mtensor);
		ChVector<> elasticForceB  = (mnodeA->J * (mtensor * (mnodeA->Amoment * (-d_BA)  ))) * (2*mnodeA->volume)  * W_BA ;
		mnodeB->UserForce += elasticForceB;

		mnodeB->t_stress.ConvertToMatrix(mtensor);
		ChVector<> elasticForceA  = (mnodeB->J * (mtensor * (mnodeB->Amoment * ( d_BA)  ))) * (2*mnodeB->volume)  * W_AB ;
		mnodeA->UserForce += elasticForceA;

		++iterproximity;
	}

}



void ChProximityContainerSPH::VariablesFbLoadForces(double factor)
{
	// do nothing

}


} // END_OF_NAMESPACE____


