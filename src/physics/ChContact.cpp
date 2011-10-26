///////////////////////////////////////////////////
//
//   ChContact.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
#include "physics/ChContact.h"
#include "physics/ChSystem.h"
#include "lcp/ChLcpConstraintTwoContactN.h"
#include "collision/ChCModelBulletBody.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


using namespace collision;
using namespace geometry;


ChContact::ChContact ()
{ 
	Nx.SetTangentialConstraintU(&Tu);
	Nx.SetTangentialConstraintV(&Tv);
}

ChContact::ChContact (	collision::ChCollisionModel* mmodA,	///< model A
						collision::ChCollisionModel* mmodB,	///< model B
						const ChLcpVariablesBody* varA, ///< pass A vars
						const ChLcpVariablesBody* varB, ///< pass B vars
						const ChFrame<>* frameA,		  ///< pass A frame
						const ChFrame<>* frameB,		  ///< pass B frame
						const ChVector<>& vpA,		  ///< pass coll.point on A
						const ChVector<>& vpB,		  ///< pass coll.point on B
						const ChVector<>& vN, 		  ///< pass coll.normal, respect to A
						double mdistance,		  ///< pass the distance (negative for penetration)
						float* mreaction_cache,	  ///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
						float mfriction,		  ///< friction coeff.
						float mcohesion,		  
						float mcompliance,		  ///< compliance = 1/stiffness [mm/N]
						float mcomplianceT		  ///< compliance = 1/stiffness [mm/N]
				)
{ 
	Nx.SetTangentialConstraintU(&Tu);
	Nx.SetTangentialConstraintV(&Tv);

	Reset(	mmodA, mmodB,
			varA, ///< pass A vars
			varB, ///< pass B vars
			frameA,		  ///< pass A frame
			frameB,		  ///< pass B frame
			vpA,		  ///< pass coll.point on A
			vpB,		  ///< pass coll.point on B
			vN, 		  ///< pass coll.normal, respect to A
			mdistance,		  ///< pass the distance (negative for penetration)
			mreaction_cache,	  ///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
			mfriction,			  ///< friction coeff.
			mcohesion,			  ///< cohesion
			mcompliance,
			mcomplianceT
				);
}

ChContact::~ChContact ()
{

}

void ChContact::Reset(	collision::ChCollisionModel* mmodA,	///< model A
						collision::ChCollisionModel* mmodB,	///< model B
					    const ChLcpVariablesBody* varA, ///< pass A vars
						const ChLcpVariablesBody* varB, ///< pass B vars
						const ChFrame<>* frameA,		  ///< pass A frame
						const ChFrame<>* frameB,		  ///< pass B frame
						const ChVector<>& vpA,		  ///< pass coll.point on A
						const ChVector<>& vpB,		  ///< pass coll.point on B
						const ChVector<>& vN, 		  ///< pass coll.normal, respect to A
						double mdistance,		  ///< pass the distance (negative for penetration)
						float* mreaction_cache,	  ///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
						float mfriction,			  ///< friction coeff.
						float mcohesion,				///< cohesion
						float mcompliance,			///< compliance = 1/stiffness [mm/N]
						float mcomplianceT			///< tang.compliance = 1/stiffness [mm/N]
				)
{
	assert (varA);
	assert (varB);
	assert (frameA);
	assert (frameB);

	this->modA = mmodA;
	this->modB = mmodB;

	Nx.SetVariables(const_cast<ChLcpVariablesBody*>(varA),const_cast<ChLcpVariablesBody*>(varB));
	Tu.SetVariables(const_cast<ChLcpVariablesBody*>(varA),const_cast<ChLcpVariablesBody*>(varB));
	Tv.SetVariables(const_cast<ChLcpVariablesBody*>(varA),const_cast<ChLcpVariablesBody*>(varB));

	Nx.SetFrictionCoefficient(mfriction);
	Nx.SetCohesion(mcohesion);
	this->compliance = mcompliance;
	this->complianceT = mcomplianceT;

	ChVector<> VN = vN;
	ChVector<double> Vx, Vy, Vz;
	ChVector<double> singul(VECT_Y);
	XdirToDxDyDz(&VN, &singul, &Vx,  &Vy, &Vz);
	contact_plane.Set_A_axis(Vx,Vy,Vz);

	this->p1 = vpA;
	this->p2 = vpB;
	this->normal = vN;
	this->norm_dist = mdistance;
	this->reactions_cache = mreaction_cache;

	ChVector<> Pl1 = frameA->TrasformParentToLocal(p1);
	ChVector<> Pl2 = frameB->TrasformParentToLocal(p2);

			// compute jacobians
	ChMatrix33<> Jx1, Jx2, Jr1, Jr2;
	ChMatrix33<> Ps1, Ps2, Jtemp;
	Ps1.Set_X_matrix(Pl1);
	Ps2.Set_X_matrix(Pl2);

	Jx1.CopyFromMatrixT(this->contact_plane);
	Jx2.CopyFromMatrixT(this->contact_plane);
	Jx1.MatrNeg();

	Jtemp.MatrMultiply(*( const_cast<ChFrame<>*>(frameA)->GetA()), Ps1);
	Jr1.MatrTMultiply(this->contact_plane, Jtemp);

	Jtemp.MatrMultiply(*( const_cast<ChFrame<>*>(frameB)->GetA()), Ps2);
	Jr2.MatrTMultiply(this->contact_plane, Jtemp);
	Jr2.MatrNeg();

	Nx.Get_Cq_a()->PasteClippedMatrix(&Jx1, 0,0, 1,3, 0,0);
	Tu.Get_Cq_a()->PasteClippedMatrix(&Jx1, 1,0, 1,3, 0,0);
	Tv.Get_Cq_a()->PasteClippedMatrix(&Jx1, 2,0, 1,3, 0,0);
	Nx.Get_Cq_a()->PasteClippedMatrix(&Jr1, 0,0, 1,3, 0,3);
	Tu.Get_Cq_a()->PasteClippedMatrix(&Jr1, 1,0, 1,3, 0,3);
	Tv.Get_Cq_a()->PasteClippedMatrix(&Jr1, 2,0, 1,3, 0,3);

	Nx.Get_Cq_b()->PasteClippedMatrix(&Jx2, 0,0, 1,3, 0,0);
	Tu.Get_Cq_b()->PasteClippedMatrix(&Jx2, 1,0, 1,3, 0,0);
	Tv.Get_Cq_b()->PasteClippedMatrix(&Jx2, 2,0, 1,3, 0,0);
	Nx.Get_Cq_b()->PasteClippedMatrix(&Jr2, 0,0, 1,3, 0,3);
	Tu.Get_Cq_b()->PasteClippedMatrix(&Jr2, 1,0, 1,3, 0,3);
	Tv.Get_Cq_b()->PasteClippedMatrix(&Jr2, 2,0, 1,3, 0,3);

	react_force = VNULL;

	//***TO DO***  C_dt? C_dtdt? (may be never used..)
}



ChCoordsys<> ChContact::GetContactCoords()
{
	ChCoordsys<> mcsys;
	ChQuaternion<float> mrot = this->contact_plane.Get_A_quaternion();
	mcsys.rot.Set(mrot.e0, mrot.e1, mrot.e2, mrot.e3);
	mcsys.pos = this->p2;
	return mcsys;
}


 


void ChContact::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	mdescriptor.InsertConstraint(&Nx);
	mdescriptor.InsertConstraint(&Tu);
	mdescriptor.InsertConstraint(&Tv); 
}

void ChContact::ConstraintsBiReset()
{
	Nx.Set_b_i(0.);
	Tu.Set_b_i(0.);
	Tv.Set_b_i(0.);
}
 
void ChContact::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	bool bounced = false;

	// Elastic Restitution model (use simple Newton model with coeffcient e=v(+)/v(-))
	// Note that this works only if the two connected items are two ChBody.

	ChModelBulletBody* bm1 = dynamic_cast<ChModelBulletBody*>(this->modA);
	ChModelBulletBody* bm2 = dynamic_cast<ChModelBulletBody*>(this->modB);
	if (bm1&&bm2)
	{
		ChBody* bb1 = bm1->GetBody();
		ChBody* bb2 = bm2->GetBody();
		double mrest_coeff =  (bb1->GetImpactC() + bb2->GetImpactC())*0.5; // = this->restitution;
		if (mrest_coeff)
		{
			//compute normal rebounce speed 
			double Ct = 0;
			
			Vector Pl1 = bb1->Point_World2Body(&this->p1);
			Vector Pl2 = bb2->Point_World2Body(&this->p2);
			Vector V1_w = bb1->PointSpeedLocalToParent(Pl1);
			Vector V2_w = bb2->PointSpeedLocalToParent(Pl2);
			Vector Vrel_w = V2_w-V1_w;
			Vector Vrel_cplane = this->contact_plane.MatrT_x_Vect(Vrel_w);
		 
			double neg_rebounce_speed = Vrel_cplane.x * mrest_coeff;
			if (neg_rebounce_speed < -  bb1->GetSystem()->GetMinBounceSpeed() )
			{
				// CASE: BOUNCE
				bounced = true;
				Nx.Set_b_i( Nx.Get_b_i() + neg_rebounce_speed );
			}
		}
	}
 
	if (!bounced)
	{
		// CASE: SETTLE (most often, and also default if two colliding items are not two ChBody)
		if (do_clamp)
			if (this->Nx.GetCohesion())
				Nx.Set_b_i( Nx.Get_b_i() + ChMin( 0.0, ChMax (factor * this->norm_dist, -recovery_clamp) ) );
			else
				Nx.Set_b_i( Nx.Get_b_i() + ChMax (factor * this->norm_dist, -recovery_clamp)  );
		else
			Nx.Set_b_i( Nx.Get_b_i() + factor * this->norm_dist  );
		
		// COMPLIANCE:
		if (this->compliance)
		{
			//  timestep is inverse of factor
			double inv_h = factor;
			double inv_hh= inv_h*inv_h;
			Nx.Set_cfm_i( (inv_hh)*this->compliance  );
			Tu.Set_cfm_i( (inv_hh)*this->complianceT );
			Tv.Set_cfm_i( (inv_hh)*this->complianceT );

			// no clamping of residual
			Nx.Set_b_i( Nx.Get_b_i() + factor * this->norm_dist  );
		}
		
	}

	//Tu.Set_b_i(Tu.Get_b_i +0.);  // nothing to add in tangential dir
	//Tv.Set_b_i(Tv.Get_b_i +0.);
}

void ChContact::ConstraintsFetch_react(double factor)
{
	// From constraints to react vector:
	react_force.x = Nx.Get_l_i() * factor;  
	react_force.y = Tu.Get_l_i() * factor;
	react_force.z = Tv.Get_l_i() * factor;
}


void  ChContact::ConstraintsLiLoadSuggestedSpeedSolution()
{
	// Fetch the last computed impulsive reactions from the persistent contact manifold (could
	// be used for warm starting the CCP speed solver): 
	if (this->reactions_cache)
	{
		Nx.Set_l_i(reactions_cache[0]);
		Tu.Set_l_i(reactions_cache[1]);  
		Tv.Set_l_i(reactions_cache[2]);
	}
	//GetLog() << "++++      " << (int)this << "  fetching N=" << (double)mn <<"\n"; 
}

void  ChContact::ConstraintsLiLoadSuggestedPositionSolution()
{
	// Fetch the last computed 'positional' reactions from the persistent contact manifold (could
	// be used for warm starting the CCP position stabilization solver):
	if (this->reactions_cache)
	{
		Nx.Set_l_i(reactions_cache[3]);
		Tu.Set_l_i(reactions_cache[4]);  
		Tv.Set_l_i(reactions_cache[5]);
	}
}

void  ChContact::ConstraintsLiFetchSuggestedSpeedSolution()
{
	// Store the last computed reactions into the persistent contact manifold (might
	// be used for warm starting CCP the speed solver):
	if (reactions_cache)
	{
		reactions_cache[0] = (float)Nx.Get_l_i();
		reactions_cache[1] = (float)Tu.Get_l_i();
		reactions_cache[2] = (float)Tv.Get_l_i();
	}
	//GetLog() << "         " << (int)this << "  storing  N=" << Nx.Get_l_i() <<"\n";
}

void  ChContact::ConstraintsLiFetchSuggestedPositionSolution()
{
	// Store the last computed 'positional' reactions into the persistent contact manifold (might
	// be used for warm starting the CCP position stabilization solver):
	if (reactions_cache)
	{
		reactions_cache[3] = (float)Nx.Get_l_i();
		reactions_cache[4] = (float)Tu.Get_l_i();
		reactions_cache[5] = (float)Tv.Get_l_i();
	}
}












} // END_OF_NAMESPACE____


