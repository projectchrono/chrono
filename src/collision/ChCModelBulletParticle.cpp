//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//////////////////////////////////////////////////
//  
//   ChCModelBulletParticle.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com 
// ------------------------------------------------
///////////////////////////////////////////////////
 
 
 
#include "ChCModelBulletParticle.h" 
#include "physics/ChIndexedParticles.h"
#include "collision/bullet/btBulletCollisionCommon.h"


namespace chrono 
{
namespace collision 
{



ChModelBulletParticle::ChModelBulletParticle()
{
	this->particles = 0;
	this->particle_id = 0;
}


ChModelBulletParticle::~ChModelBulletParticle()
{
}


void ChModelBulletParticle::SetParticle(ChIndexedParticles* mpa, unsigned int id)
{
	this->particles = mpa;
	this->particle_id = id;
}


void ChModelBulletParticle::SyncPosition()
{
	assert(particles);

	ChParticleBase* ppointer = &particles->GetParticle(this->particle_id);
	
	assert(ppointer);
	assert(particles->GetSystem());

	bt_collision_object->getWorldTransform().setOrigin(btVector3(
								(btScalar)ppointer->GetPos().x,
								(btScalar)ppointer->GetPos().y,
								(btScalar)ppointer->GetPos().z));
	ChMatrix33<>* rA = ppointer->GetA();
	btMatrix3x3 basisA( (btScalar)(*rA)(0,0), (btScalar)(*rA)(0,1), (btScalar)(*rA)(0,2),
						(btScalar)(*rA)(1,0), (btScalar)(*rA)(1,1), (btScalar)(*rA)(1,2),
						(btScalar)(*rA)(2,0), (btScalar)(*rA)(2,1), (btScalar)(*rA)(2,2));
	bt_collision_object->getWorldTransform().setBasis(basisA);
}







} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

