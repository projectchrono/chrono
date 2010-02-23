#ifndef CH_NOCUDA 

///////////////////////////////////////////////////
//
//   ChLinkGPUcontact.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
#include "physics/ChLinkGPUContact.h"
#include "physics/ChSystem.h"
#include "physics/ChCollide.h"
#include "lcp/ChLcpConstraintTwoGPUcontN.h"

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
ChClassRegister<ChLinkGPUcontact> a_registration_ChLinkGPUcontact;
  
ChLinkGPUcontact::ChLinkGPUcontact ()
{ 
	Nx.SetTangentialConstraintU(&Tu);
	Nx.SetTangentialConstraintV(&Tv);
}

ChLinkGPUcontact::ChLinkGPUcontact (ChCollisionPair* mpair, 
									ChBody* mbody1, 
									ChBody* mbody2)
{ 
	assert (mpair);
	assert (mbody1 && mbody2);

	Nx.SetTangentialConstraintU(&Tu);
	Nx.SetTangentialConstraintV(&Tv);

	Reset(mpair, mbody1, mbody2);
}

ChLinkGPUcontact::~ChLinkGPUcontact ()
{

}

void ChLinkGPUcontact::Reset(ChCollisionPair* mpair, 
							  ChBody* mbody1, 
							  ChBody* mbody2)
{
	this->Body1 = mbody1;
	this->Body2 = mbody2;

	Nx.SetVariables(&this->Body1->Variables(),&this->Body2->Variables());
	Tu.SetVariables(&this->Body1->Variables(),&this->Body2->Variables()); //unuseful??
	Tv.SetVariables(&this->Body1->Variables(),&this->Body2->Variables()); //unuseful??

	Nx.SetP1(mpair->p1);
	Nx.SetP2(mpair->p2);
	Nx.SetNormal(mpair->normal);
	Nx.SetContactCache(mpair->reactions_cache);

	Nx.SetFrictionCoefficient(this->sta_fri);
}

void ChLinkGPUcontact::Copy(ChLinkGPUcontact* source)
{
    // first copy the parent class data...
    //
    ChLinkContact::Copy(source);

    // copy custom data:
	ChCollisionPair mpair;
	 mpair.p1 = source->Nx.GetP1();
	 mpair.p2 = source->Nx.GetP2();
	 mpair.normal = source->Nx.GetNormal();
	 mpair.reactions_cache = source->Nx.GetContactCache();
	this->Reset(&mpair, source->GetBody1(), source->GetBody2() );
}

ChLink* ChLinkGPUcontact::new_Duplicate ()
{
    ChLinkGPUcontact* m_l;
    m_l = new ChLinkGPUcontact;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}

ChCoordsys<> ChLinkGPUcontact::GetLinkRelativeCoords()
{
	Vector Pl2 = Body2->Point_World2Body(&this->Nx.GetP2());
	ChMatrix33<> m12; 
	ChMatrix33<float> contact_plane;
	ChVector<float> Vx, Vy, Vz;
	ChVector<float> Vsingul(VECT_Y);
	XdirToDxDyDz(&this->Nx.GetNormal(), &Vsingul, &Vx,  &Vy, &Vz);
	contact_plane.Set_A_axis(Vx,Vy,Vz);
	m12.MatrTMultiply(*Body2->GetA(), contact_plane);
	Quaternion Ql2 = m12.Get_A_quaternion();
	return ChCoordsys<>(Pl2, Ql2); 
}



double	 ChLinkGPUcontact::GetContactDistance()
{
	return Vdot(Nx.GetNormal(), Nx.GetP2()-Nx.GetP1());
}


////////// LCP INTERFACES ////


void ChLinkGPUcontact::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	if (!this->IsActive())
		return;

	mdescriptor.InsertConstraint(&Nx);
	mdescriptor.InsertConstraint(&Tu);
	mdescriptor.InsertConstraint(&Tv); 
}

void ChLinkGPUcontact::ConstraintsBiReset()
{
	Nx.Set_b_i(0.);
	Tu.Set_b_i(0.);
	Tv.Set_b_i(0.);
}
 
void ChLinkGPUcontact::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	// DO NOTHING: will be done on the GPU
}

void ChLinkGPUcontact::ConstraintsBiLoad_Ct(double factor)
{
	// DO NOTHING: will be done on the GPU
}


void ChLinkGPUcontact::ConstraintsLoadJacobians()
{
	// DO NOTHING: will be done on the GPU 
}
 

void ChLinkGPUcontact::ConstraintsFetch_react(double factor)
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

void  ChLinkGPUcontact::ConstraintsLiLoadSuggestedSpeedSolution()
{
	// Fetch the last computed impulsive reactions from the persistent contact manifold (could
	// be used for warm starting the CCP speed solver):
	Nx.Set_l_i( (Nx.GetContactCache())[0] );
	Tu.Set_l_i( (Nx.GetContactCache())[1] );  
	Tv.Set_l_i( (Nx.GetContactCache())[2] );
	//GetLog() << "++++      " << (int)this << "  fetching N=" << (double)mn <<"\n"; 
}

void  ChLinkGPUcontact::ConstraintsLiLoadSuggestedPositionSolution()
{
	// Fetch the last computed 'positional' reactions from the persistent contact manifold (could
	// be used for warm starting the CCP position stabilization solver):
	Nx.Set_l_i( (Nx.GetContactCache())[3] );
	Tu.Set_l_i( (Nx.GetContactCache())[4] );  
	Tv.Set_l_i( (Nx.GetContactCache())[5] );
}

void  ChLinkGPUcontact::ConstraintsLiFetchSuggestedSpeedSolution()
{
	// Store the last computed reactions into the persistent contact manifold (might
	// be used for warm starting CCP the speed solver):
	(Nx.GetContactCache())[0] = (float)Nx.Get_l_i();
	(Nx.GetContactCache())[1] = (float)Tu.Get_l_i();
	(Nx.GetContactCache())[2] = (float)Tv.Get_l_i();
	//GetLog() << "         " << (int)this << "  storing  N=" << Nx.Get_l_i() <<"\n";
}

void  ChLinkGPUcontact::ConstraintsLiFetchSuggestedPositionSolution()
{
	// Store the last computed 'positional' reactions into the persistent contact manifold (might
	// be used for warm starting the CCP position stabilization solver):
	(Nx.GetContactCache())[3] = (float)Nx.Get_l_i();
	(Nx.GetContactCache())[4] = (float)Tu.Get_l_i();
	(Nx.GetContactCache())[5] = (float)Tv.Get_l_i();
}




} // END_OF_NAMESPACE____




#endif  // end of ! CH_NOCUDA
 
