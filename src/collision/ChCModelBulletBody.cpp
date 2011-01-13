//////////////////////////////////////////////////
//  
//   ChCModelBulletBody.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com 
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
 
#include "ChCModelBulletBody.h" 
#include "physics/ChBody.h"
#include "collision/bullet/btBulletCollisionCommon.h"


namespace chrono 
{
namespace collision 
{



ChModelBulletBody::ChModelBulletBody()
{
	mbody = 0;
}


ChModelBulletBody::~ChModelBulletBody()
{
}

void ChModelBulletBody::SyncPosition()
{
	ChBody* bpointer = GetBody();
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

