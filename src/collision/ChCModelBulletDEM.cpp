//////////////////////////////////////////////////
//  
//   ChCModelBulletBody.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com 
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
 
#include "ChCModelBulletDEM.h" 
#include "physics/ChBodyDEM.h"
#include "collision/bullet/btBulletCollisionCommon.h"


namespace chrono 
{
namespace collision 
{



ChModelBulletDEM::ChModelBulletDEM()
{
	mbody = 0;
}


ChModelBulletDEM::~ChModelBulletDEM()
{
}

void ChModelBulletDEM::SyncPosition()
{
	ChBodyDEM* bpointer = GetBody();
	assert(bpointer);
	//assert(bpointer->GetSystem());

	bt_collision_object->getWorldTransform().setOrigin(btVector3(
								(btScalar)bpointer->GetPos().x,
								(btScalar)bpointer->GetPos().y,
								(btScalar)bpointer->GetPos().z));
	ChMatrix33<>* rA = bpointer->GetA();
	btMatrix3x3 basisA( (btScalar)(*rA)(0,0), (btScalar)(*rA)(0,1), (btScalar)(*rA)(0,2),
						(btScalar)(*rA)(1,0), (btScalar)(*rA)(1,1), (btScalar)(*rA)(1,2),
						(btScalar)(*rA)(2,0), (btScalar)(*rA)(2,1), (btScalar)(*rA)(2,2));
	bt_collision_object->getWorldTransform().setBasis(basisA);
}







} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

