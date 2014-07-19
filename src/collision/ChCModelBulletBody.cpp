//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//////////////////////////////////////////////////
//  
//   ChCModelBulletBody.cpp
//
// ------------------------------------------------
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
	
	// To support also the new generic ChBodyAuxRef, the collision
	// shape is not placed respect to COG csys, but rather REF csys (that are
	// the same in basic ChBody, anyway)
	const ChFrame<>& framepointer = bpointer->GetFrame_REF_to_abs();

	bt_collision_object->getWorldTransform().setOrigin(btVector3(
	                    (btScalar)framepointer.GetPos().x,
	                    (btScalar)framepointer.GetPos().y,
	                    (btScalar)framepointer.GetPos().z));
	const ChMatrix33<>& rA = framepointer.GetA();
	btMatrix3x3 basisA( (btScalar)rA(0,0), (btScalar)rA(0,1), (btScalar)rA(0,2),
	                    (btScalar)rA(1,0), (btScalar)rA(1,1), (btScalar)rA(1,2),
	                    (btScalar)rA(2,0), (btScalar)rA(2,1), (btScalar)rA(2,2));
	bt_collision_object->getWorldTransform().setBasis(basisA);
}







} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

