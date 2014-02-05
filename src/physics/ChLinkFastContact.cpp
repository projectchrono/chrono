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
//   ChLinkFastContact.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
#include "physics/ChLinkFastContact.h"
#include "physics/ChSystem.h"
//#include "physics/ChCollide.h"
#include "lcp/ChLcpConstraintTwoContactN.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


using namespace collision;
using namespace geometry;






////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//
//   CLASS FOR FAST CONTACT
//
//

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkFastContact> a_registration_ChLinkFastContact;
 

ChLinkFastContact::ChLinkFastContact ()
{ 
	Nx.SetTangentialConstraintU(&Tu);
	Nx.SetTangentialConstraintV(&Tv);
}

ChLinkFastContact::ChLinkFastContact (ChCollisionPair* mpair,
									  ChBodyFrame* mbody1, 
									  ChBodyFrame* mbody2)
{ 
	Nx.SetTangentialConstraintU(&Tu);
	Nx.SetTangentialConstraintV(&Tv);

	Reset(mpair,mbody1,mbody2);
}

ChLinkFastContact::~ChLinkFastContact ()
{

}

void ChLinkFastContact::Reset(ChCollisionPair* mpair, 
							  ChBodyFrame* mbody1, 
							  ChBodyFrame* mbody2)
{
	assert (mpair);
	assert (mbody1);
	assert (mbody2);

	this->Body1 = mbody1;
	this->Body2 = mbody2;

	Nx.SetVariables(&this->Body1->Variables(),&this->Body2->Variables());
	Tu.SetVariables(&this->Body1->Variables(),&this->Body2->Variables());
	Tv.SetVariables(&this->Body1->Variables(),&this->Body2->Variables());

	this->collision_pair = *mpair;

	ChVector<float> Vx, Vy, Vz;
	ChVector<float> singul(VECT_Y);
	XdirToDxDyDz(&this->collision_pair.normal, &singul, &Vx,  &Vy, &Vz);
	contact_plane.Set_A_axis(Vx,Vy,Vz);
}


void ChLinkFastContact::Copy(ChLinkFastContact* source)
{
    // first copy the parent class data...
    //
    ChLinkContact::Copy(source);
	
	this->Reset(&source->collision_pair, source->GetBody1(), source->GetBody2() );
}

ChLink* ChLinkFastContact::new_Duplicate ()
{
    ChLinkFastContact* m_l;
    m_l = new ChLinkFastContact(&this->collision_pair, this->Body1, this->Body2);  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}

ChCoordsys<> ChLinkFastContact::GetLinkRelativeCoords()
{
	Vector Pl2 = Body2->TrasformPointParentToLocal(this->collision_pair.p2);
	ChMatrix33<> m12; 
	m12.MatrTMultiply(*Body2->GetA(), contact_plane);
	Quaternion Ql2 = m12.Get_A_quaternion();
	return ChCoordsys<>(Pl2, Ql2); 
}


void ChLinkFastContact::Update (double mytime)
{
    // Inherit time changes of parent class (ChLink), basically doing nothing :)
    ChLink::UpdateTime(mytime);

		// contact point in body coordinates
	Vector Pl1 = Body1->TrasformPointParentToLocal(this->collision_pair.p1);
	Vector Pl2 = Body2->TrasformPointParentToLocal(this->collision_pair.p2);

		// compute friction
	//double mfriction_coeff = (Body1->GetSfriction() + Body2->GetSfriction())*0.5;

	Nx.SetFrictionCoefficient(this->sta_fri);
	
		// compute jacobians
	ChMatrix33<> Jx1, Jx2, Jr1, Jr2;
	ChMatrix33<> Ps1, Ps2, Jtemp;
	Ps1.Set_X_matrix(Pl1);
	Ps2.Set_X_matrix(Pl2);

	Jx1.CopyFromMatrixT(this->contact_plane);
	Jx2.CopyFromMatrixT(this->contact_plane);
	Jx1.MatrNeg();

	Jtemp.MatrMultiply(*Body1->GetA(), Ps1);
	Jr1.MatrTMultiply(this->contact_plane, Jtemp);

	Jtemp.MatrMultiply(*Body2->GetA(), Ps2);
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

	//***TO DO***  C_dt? C_dtdt? (may be never used..)
}




////////// LCP INTERFACES ////


void ChLinkFastContact::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	if (!this->IsActive())
		return;

	mdescriptor.InsertConstraint(&Nx);
	mdescriptor.InsertConstraint(&Tu);
	mdescriptor.InsertConstraint(&Tv); 
}

void ChLinkFastContact::ConstraintsBiReset()
{
	Nx.Set_b_i(0.);
	Tu.Set_b_i(0.);
	Tv.Set_b_i(0.);
}
 
void ChLinkFastContact::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	if (!this->IsActive())
		return;

	//Tu.Set_b_i(Tu.Get_b_i +0.);  //increment nothing
	//Tv.Set_b_i(Tv.Get_b_i +0.);

	if (do_clamp)
		Nx.Set_b_i( Nx.Get_b_i() + ChMax (factor * this->collision_pair.norm_dist, -recovery_clamp)  );
	else
		Nx.Set_b_i( Nx.Get_b_i() + factor * this->collision_pair.norm_dist  );
}

void ChLinkFastContact::ConstraintsBiLoad_Ct(double factor)
{
	if (!this->IsActive())
		return;

	// compute normal rebounce speed (use simple Newton model)
	double Ct = 0;
	/* ***DEACTIVATED***
	Vector Pl1 = Body1->Point_World2Body(this->collision_pair.p1);
	Vector Pl2 = Body2->Point_World2Body(this->collision_pair.p2);
	Vector V1_w = Body1->PointSpeedLocalToParent(Pl1);
	Vector V2_w = Body2->PointSpeedLocalToParent(Pl2);
	Vector Vrel_w = V2_w-V1_w;
	Vector Vrel_cplane = this->contact_plane.MatrT_x_Vect(Vrel_w);
 
	double mrest_coeff = this->restitution; // (Body1->GetImpactC() + Body2->GetImpactC())*0.5;
	double neg_rebounce_speed = Vrel_cplane.x * mrest_coeff;
	if (neg_rebounce_speed < - ((ChSystem*)this->system)->GetMinBounceSpeed() )
		Ct= neg_rebounce_speed;
	else
		Ct= 0.0;

	//Tu.Set_b_i(Tu.Get_b_i +0.);  //increment nothing
	//Tv.Set_b_i(Tv.Get_b_i +0.);
	Nx.Set_b_i( Nx.Get_b_i() + factor * Ct );
	*/
}

/*


void ChLinkFastContact::ConstraintsFbLoadForces(double factor)
{
	// no forces
}
*/

void ChLinkFastContact::ConstraintsLoadJacobians()
{
	// already loaded when doing Update (which used the matrices of the scalar constraint objects)

	  
}
 

void ChLinkFastContact::ConstraintsFetch_react(double factor)
{
	// From constraints to react vector:
	react_force.x = Nx.Get_l_i() * factor;  
	react_force.y = Tu.Get_l_i() * factor;
	react_force.z = Tv.Get_l_i() * factor;

	react_torque = VNULL;
}


// 
// Following functions are for exploiting the contact persistence
//

void  ChLinkFastContact::ConstraintsLiLoadSuggestedSpeedSolution()
{
	// Fetch the last computed impulsive reactions from the persistent contact manifold (could
	// be used for warm starting the CCP speed solver):
	float mn,mu,mv;
	this->collision_pair.CacheFetchSpeedSolutionFromManifold(mn,mu,mv); 
	Nx.Set_l_i(mn);
	Tu.Set_l_i(mu);  
	Tv.Set_l_i(mv);
	//GetLog() << "++++      " << (int)this << "  fetching N=" << (double)mn <<"\n"; 
}

void  ChLinkFastContact::ConstraintsLiLoadSuggestedPositionSolution()
{
	// Fetch the last computed 'positional' reactions from the persistent contact manifold (could
	// be used for warm starting the CCP position stabilization solver):
	float mn,mu,mv;
	this->collision_pair.CacheFetchPositionSolutionFromManifold(mn,mu,mv); 
	Nx.Set_l_i(mn);
	Tu.Set_l_i(mu);  
	Tv.Set_l_i(mv);
}

void  ChLinkFastContact::ConstraintsLiFetchSuggestedSpeedSolution()
{
	// Store the last computed reactions into the persistent contact manifold (might
	// be used for warm starting CCP the speed solver):
	this->collision_pair.CacheStoreSpeedSolutionIntoManifold((float)Nx.Get_l_i(), (float)Tu.Get_l_i(), (float)Tv.Get_l_i());
	//GetLog() << "         " << (int)this << "  storing  N=" << Nx.Get_l_i() <<"\n";
}

void  ChLinkFastContact::ConstraintsLiFetchSuggestedPositionSolution()
{
	// Store the last computed 'positional' reactions into the persistent contact manifold (might
	// be used for warm starting the CCP position stabilization solver):
	this->collision_pair.CacheStorePositionSolutionIntoManifold((float)Nx.Get_l_i(), (float)Tu.Get_l_i(), (float)Tv.Get_l_i());
}




} // END_OF_NAMESPACE____


