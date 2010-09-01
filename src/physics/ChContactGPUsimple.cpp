#ifdef CH_UNIT_CUDA 

///////////////////////////////////////////////////
//
//   ChContactGPUsimple.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
#include "physics/ChContactGPUsimple.h"
#include "physics/ChSystem.h"
#include "lcp/ChLcpConstraintTwoContactN.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


using namespace collision;
using namespace geometry;


ChContactGPUsimple::ChContactGPUsimple ()
{ 
	Nx.SetTangentialConstraintU(&Tu);
	Nx.SetTangentialConstraintV(&Tv);
}

ChContactGPUsimple::ChContactGPUsimple (	collision::ChCollisionModel* mmodA,	///< model A
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
						float mfriction			  ///< friction coeff.
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
			mfriction			  ///< friction coeff.
				);
}

ChContactGPUsimple::~ChContactGPUsimple ()
{

}

void ChContactGPUsimple::Reset(	collision::ChCollisionModel* mmodA,	///< model A
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
						float mfriction			  ///< friction coeff.
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

	ChVector<double> mp1(vpA);
	ChVector<double> mp2(vpB);
	ChVector<float> mn(vN);

	Nx.SetP1(mp1);
	Nx.SetP2(mp2);
	Nx.SetNormal(mn);
	Nx.SetContactCache(mreaction_cache);
	Nx.SetFrictionCoefficient(mfriction);

	// No need to compute jacobians, because Dan's postprocessing kernel will take care of making them in GPU.

	react_force = VNULL;

}



ChCoordsys<> ChContactGPUsimple::GetContactCoords()
{
	ChVector<float> Vx, Vy, Vz;
	ChVector<float> Vsingul(VECT_Y);
	ChMatrix33<float> contact_plane;
	XdirToDxDyDz(&this->Nx.GetNormal(), &Vsingul, &Vx,  &Vy, &Vz);
	contact_plane.Set_A_axis(Vx,Vy,Vz);
	ChCoordsys<> mcsys;
	ChQuaternion<float> mrot = contact_plane.Get_A_quaternion();
	mcsys.rot.Set(mrot.e0, mrot.e1, mrot.e2, mrot.e3);
	mcsys.pos = this->GetContactP1();
	return mcsys;
}


 


void ChContactGPUsimple::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	mdescriptor.InsertConstraint(&Nx);
	mdescriptor.InsertConstraint(&Tu);
	mdescriptor.InsertConstraint(&Tv); 
}

void ChContactGPUsimple::ConstraintsBiReset()
{
	Nx.Set_b_i(0.);
	Tu.Set_b_i(0.);
	Tv.Set_b_i(0.);
}
 
void ChContactGPUsimple::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	// DO NOTHING: will be done on the GPU with Dan's preprocessing
}

void ChContactGPUsimple::ConstraintsFetch_react(double factor)
{
	// From constraints to react vector:
	react_force.x = Nx.Get_l_i() * factor;  
	react_force.y = Tu.Get_l_i() * factor;
	react_force.z = Tv.Get_l_i() * factor;
}


void  ChContactGPUsimple::ConstraintsLiLoadSuggestedSpeedSolution()
{
	// Fetch the last computed impulsive reactions from the persistent contact manifold (could
	// be used for warm starting the CCP speed solver): 
	if (Nx.GetContactCache())
	{
		Nx.Set_l_i((Nx.GetContactCache())[0]);
		Tu.Set_l_i((Nx.GetContactCache())[1]);  
		Tv.Set_l_i((Nx.GetContactCache())[2]);
	}
	//GetLog() << "++++      " << (int)this << "  fetching N=" << (double)mn <<"\n"; 
}

void  ChContactGPUsimple::ConstraintsLiLoadSuggestedPositionSolution()
{
	// Fetch the last computed 'positional' reactions from the persistent contact manifold (could
	// be used for warm starting the CCP position stabilization solver):
	if (Nx.GetContactCache())
	{
		Nx.Set_l_i((Nx.GetContactCache())[3]);
		Tu.Set_l_i((Nx.GetContactCache())[4]);  
		Tv.Set_l_i((Nx.GetContactCache())[5]);
	}
}

void  ChContactGPUsimple::ConstraintsLiFetchSuggestedSpeedSolution()
{
	// Store the last computed reactions into the persistent contact manifold (might
	// be used for warm starting CCP the speed solver):
	if (Nx.GetContactCache())
	{
		(Nx.GetContactCache())[0] = (float)Nx.Get_l_i();
		(Nx.GetContactCache())[1] = (float)Tu.Get_l_i();
		(Nx.GetContactCache())[2] = (float)Tv.Get_l_i();
	}
	//GetLog() << "         " << (int)this << "  storing  N=" << Nx.Get_l_i() <<"\n";
}

void  ChContactGPUsimple::ConstraintsLiFetchSuggestedPositionSolution()
{
	// Store the last computed 'positional' reactions into the persistent contact manifold (might
	// be used for warm starting the CCP position stabilization solver):
	if (Nx.GetContactCache())
	{
		(Nx.GetContactCache())[3] = (float)Nx.Get_l_i();
		(Nx.GetContactCache())[4] = (float)Tu.Get_l_i();
		(Nx.GetContactCache())[5] = (float)Tv.Get_l_i();
	}
}




} // END_OF_NAMESPACE____



#endif  // end of   CH_UNIT_CUDA
 


