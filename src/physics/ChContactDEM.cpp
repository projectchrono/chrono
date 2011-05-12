///////////////////////////////////////////////////
//
//   ChContactDEM.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
#include "physics/ChContactDEM.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyDEM.h"
#include "lcp/ChLcpConstraintTwoContactN.h"
#include "collision/ChCModelBulletDEM.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


using namespace collision;
using namespace geometry;


ChContactDEM::ChContactDEM ()
{ 
	
}

ChContactDEM::ChContactDEM (		collision::ChCollisionModel* mmodA,	///< model A
						collision::ChCollisionModel* mmodB,	///< model B
						const ChLcpVariablesBody* varA, ///< pass A vars
						const ChLcpVariablesBody* varB, ///< pass B vars
						const ChFrame<>* frameA,		///< pass A frame
						const ChFrame<>* frameB,		///< pass B frame
						const ChVector<>& vpA,			///< pass coll.point on A, in absolute coordinates
						const ChVector<>& vpB,			///< pass coll.point on B, in absolute coordinates
						const ChVector<>& vN, 			///< pass coll.normal, respect to A, in absolute coordinates
						double mdistance,				///< pass the distance (negative for penetration)
						float* mreaction_cache,			///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
						double  mkn,				///< spring coeff.
						double  mgn				///< damping coeff.
				)
{ 
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
			mkn,
			mgn
				);
}

ChContactDEM::~ChContactDEM ()
{

}

void ChContactDEM::Reset(	collision::ChCollisionModel* mmodA,	///< model A
							collision::ChCollisionModel* mmodB,	///< model B
							const ChLcpVariablesBody* varA, ///< pass A vars
							const ChLcpVariablesBody* varB, ///< pass B vars
							const ChFrame<>* frameA,		///< pass A frame
							const ChFrame<>* frameB,		///< pass B frame
							const ChVector<>& vpA,			///< pass coll.point on A, in absolute coordinates
							const ChVector<>& vpB,			///< pass coll.point on B, in absolute coordinates
							const ChVector<>& vN, 			///< pass coll.normal, respect to A, in absolute coordinates
							double mdistance,				///< pass the distance (negative for penetration)
							float* mreaction_cache,			///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
							double  mkn,				///< spring coeff.
							double  mgn				///< damping coeff.
				)
{
	assert (varA);
	assert (varB);
	assert (frameA);
	assert (frameB);


	this->modA = mmodA;
	this->modB = mmodB;

	ChVector<> VN = vN;
	ChVector<double> Vx, Vy, Vz;
	ChVector<double> singul(VECT_Y);
	XdirToDxDyDz(&VN, &singul, &Vx,  &Vy, &Vz);
	contact_plane.Set_A_axis(Vx,Vy,Vz);

	this->p1 = vpA;
	this->p2 = vpB;
	this->normal = vN;
	this->norm_dist = mdistance;
	this->gnn=mgn;
	this->knn=mkn;

	react_force = VNULL;
	if (mdistance<0)
	{
		ChModelBulletDEM* modelA = (ChModelBulletDEM*)mmodA;
		ChBodyDEM* bodyA = modelA->GetBody();
		ChModelBulletDEM* modelB = (ChModelBulletDEM*)mmodB;
		ChBodyDEM* bodyB = modelB->GetBody();

		//spring force
		react_force+=mkn*mdistance*vN;

		//damping force - How to compute v_n?
		ChVector<> local_pA = bodyA->Point_World2Body(&p1);
		ChVector<> local_pB = bodyB->Point_World2Body(&p2);

		//ChVector<> v_BA = (bodyB->GetPos_dt())-(bodyA->GetPos_dt());
		ChVector<> v_BA = (bodyB->RelPoint_AbsSpeed(&local_pB))-(bodyA->RelPoint_AbsSpeed(&local_pA));
		ChVector<> v_n = (v_BA.Dot(vN))*vN;
		double mmm = (bodyA->GetMass())+(bodyB->GetMass());
		react_force+=mgn*(mmm/4)*v_n;

		//bodyA->AccumulateForce(react_force);
		//bodyB->AccumulateForce(-react_force);
	}
}



ChCoordsys<> ChContactDEM::GetContactCoords()
{
	ChCoordsys<> mcsys;
	ChQuaternion<float> mrot = this->contact_plane.Get_A_quaternion();
	mcsys.rot.Set(mrot.e0, mrot.e1, mrot.e2, mrot.e3);
	mcsys.pos = this->p2;
	return mcsys;
}

} // END_OF_NAMESPACE____


