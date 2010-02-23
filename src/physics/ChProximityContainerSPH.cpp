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


void ChProximityContainerSPH::VariablesFbLoadForces(double factor)
{
	ChVector<> Xforce;
	
	// Per-node data initialization.  
	// (but looping on edges) Btw: might be better to move it in the ChMatterSPH class, 
	// looping on nodes directly, to avoid the problem of multiple iitializations if multiple edges 
	// act on same node.

	std::list<ChProximitySPH*>::iterator iterproximity = proximitylist.begin();
	while(iterproximity != proximitylist.end())
	{
		ChMatterSPH* mmatA = (ChMatterSPH*)(*iterproximity)->GetModelA()->GetPhysicsItem();
		ChMatterSPH* mmatB = (ChMatterSPH*)(*iterproximity)->GetModelB()->GetPhysicsItem();
		ChNodeSPH* mnodeA =(ChNodeSPH*) &mmatA->GetNode ( ((ChModelBulletNode*)(*iterproximity)->GetModelA())->GetNodeId()  );
		ChNodeSPH* mnodeB =(ChNodeSPH*) &mmatB->GetNode ( ((ChModelBulletNode*)(*iterproximity)->GetModelB())->GetNodeId()  );
		
		mnodeA->J.FillElem(0.0);
		mnodeA->Amoment.FillElem(0.0);
		mnodeA->t_strain.FillElem(0.0);
		mnodeA->t_stress.FillElem(0.0);
		mnodeA->m_v = VNULL;
		mnodeA->edge_processed = false;
		mnodeA->UserForce = VNULL;

		mnodeB->J.FillElem(0.0);
		mnodeB->Amoment.FillElem(0.0);
		mnodeB->t_strain.FillElem(0.0);
		mnodeB->t_stress.FillElem(0.0);
		mnodeB->m_v = VNULL;
		mnodeB->edge_processed = false;
		mnodeB->UserForce = VNULL;
		
		++iterproximity;
	}


	// Per-edge data computation

	iterproximity = proximitylist.begin();
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


		ChVector<> m_inc_BA = d_BA * W_BA;
		mnodeA->m_v = mnodeA->m_v + m_inc_BA; // increment the m_v vector
		
		ChVector<> dwg;					// increment the J matrix
		dwg = m_inc_BA * g_BA.x;
		mnodeA->J.PasteSumVector(dwg,0,0);
		dwg = m_inc_BA * g_BA.y;
		mnodeA->J.PasteSumVector(dwg,0,1);
		dwg = m_inc_BA * g_BA.z;
		mnodeA->J.PasteSumVector(dwg,0,2);		

		ChVector<> m_inc_AB = (-d_BA) * W_AB;
		mnodeB->m_v = mnodeB->m_v + m_inc_AB; // increment the m_v vector
	
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

	// Per-node data computation  
	// (but looping on edges) Btw: might be better to move it in the ChMatterSPH class, 
	// looping on nodes directly, to avoid the mnodeA->edge_processed trick..

	iterproximity = proximitylist.begin();
	while(iterproximity != proximitylist.end())
	{
		ChMatterSPH* mmatA = (ChMatterSPH*)(*iterproximity)->GetModelA()->GetPhysicsItem();
		ChMatterSPH* mmatB = (ChMatterSPH*)(*iterproximity)->GetModelB()->GetPhysicsItem();
		ChNodeSPH* mnodeA =(ChNodeSPH*) &mmatA->GetNode ( ((ChModelBulletNode*)(*iterproximity)->GetModelA())->GetNodeId()  );
		ChNodeSPH* mnodeB =(ChNodeSPH*) &mmatB->GetNode ( ((ChModelBulletNode*)(*iterproximity)->GetModelB())->GetNodeId()  );

		if (!mnodeA->edge_processed)	// avoid multiple calls on single node from multiple concurrent edges
		{
			// Compute A inverse
			ChMatrix33<> M_tmp = mnodeA->Amoment;
			double det = M_tmp.FastInvert(&mnodeA->Amoment);
			if (fabs(det)<0.00001) 
			{
				mnodeA->Amoment.FillElem(0); // deactivate if not possible to invert
			}

			// Compute J = ( A^-1 * [dwg | dwg | dwg] )' + I
			M_tmp.MatrMultiply(mnodeA->Amoment, mnodeA->J);
			M_tmp.Element(0,0) +=1; 
			M_tmp.Element(1,1) +=1;
			M_tmp.Element(2,2) +=1;	
			mnodeA->J.CopyFromMatrixT(M_tmp);

			// Compute strain tensor  epsilon = J'*J - I
			ChMatrix33<> mtensor;
			mtensor.MatrMultiply(M_tmp, mnodeA->J);
			mtensor.Element(0,0)-=1;
			mtensor.Element(1,1)-=1;
			mtensor.Element(2,2)-=1;

			ChStrainTensor<> elasticstrain;
			mnodeA->t_strain.ConvertFromMatrix(mtensor);
			elasticstrain.MatrSub( mnodeA->t_strain , mnodeA->p_strain);
			mmatA->GetMaterial().ComputeElasticStress(mnodeA->t_stress, elasticstrain);
			mnodeA->t_stress.ConvertToMatrix(mtensor);

			Xforce      = (mnodeA->J * (mtensor * (mnodeA->Amoment * mnodeA->m_v))) * (2*mnodeA->volume);
			
			mnodeA->Variables().Get_fb().PasteSumVector( Xforce * factor ,0,0);
			//mnodeA->UserForce = Xforce;
			mnodeA->Variables().Get_fb().PasteSumVector( mnodeA->UserForce * factor ,0,0); // visco effects


			// absorb deformation in plastic tensor, as in Muller paper.
		    mnodeA->p_strain.MatrSub(mnodeA->p_strain , mnodeA->t_strain);
		    mnodeA->t_strain.FillElem(0);
		    mnodeA->pos_ref = mnodeA->pos;

			mnodeA->edge_processed = true;
		}

		if (!mnodeB->edge_processed)	// avoid multiple calls on single node from multiple concurrent edges
		{
			// Compute A inverse
			ChMatrix33<> M_tmp = mnodeB->Amoment;
			double det = M_tmp.FastInvert(&mnodeB->Amoment);
			if (fabs(det)<0.00001) 
			{
				mnodeB->Amoment.FillElem(0); // deactivate if not possible to invert
			}
			// Compute J = ( A^-1 * [dwg | dwg | dwg] )' + I
			M_tmp.MatrMultiply(mnodeB->Amoment, mnodeB->J);
			M_tmp.Element(0,0) +=1; 
			M_tmp.Element(1,1) +=1;
			M_tmp.Element(2,2) +=1;	
			mnodeB->J.CopyFromMatrixT(M_tmp);

			// Compute strain tensor  epsilon = J'*J - I
			ChMatrix33<> mtensor;
			mtensor.MatrMultiply(M_tmp, mnodeB->J);
			mtensor.Element(0,0)-=1; 
			mtensor.Element(1,1)-=1;
			mtensor.Element(2,2)-=1;

			ChStrainTensor<> elasticstrain;
			mnodeB->t_strain.ConvertFromMatrix(mtensor);
			elasticstrain.MatrSub( mnodeB->t_strain , mnodeB->p_strain);
			mmatB->GetMaterial().ComputeElasticStress(mnodeB->t_stress, elasticstrain);
			mnodeB->t_stress.ConvertToMatrix(mtensor);

			Xforce      = (mnodeB->J * (mtensor * (mnodeB->Amoment * mnodeB->m_v))) * (2*mnodeB->volume);
			
			mnodeB->Variables().Get_fb().PasteSumVector( Xforce * factor ,0,0);
			//mnodeB->UserForce = Xforce;
			mnodeB->Variables().Get_fb().PasteSumVector( mnodeB->UserForce * factor ,0,0); // visco effects

			// absorb deformation in plastic tensor, as in Muller paper.
		    mnodeB->p_strain.MatrSub(mnodeB->p_strain , mnodeB->t_strain);
		    mnodeB->t_strain.FillElem(0);
		    mnodeB->pos_ref = mnodeB->pos;

			mnodeB->edge_processed = true;
		}

		++iterproximity;
	}

	iterproximity = proximitylist.begin();
	while(iterproximity != proximitylist.end())
	{

		++iterproximity;
	}

	// add applied forces and torques (and also the gyroscopic torque!) to 'fb' vector
	//this->variables.Get_fb().PasteSumVector( Xforce * factor ,0,0);
	//this->variables.Get_fb().PasteSumVector((Xtorque + gyro)* factor ,3,0);
}


} // END_OF_NAMESPACE____


