// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// ChronoParallel unit test for MPR collision detection
// =============================================================================

#include <stdio.h>
#include <vector>
#include <cmath>
#include "unit_testing.h"
#include "chrono_parallel/collision/ChCNarrowphaseMPR.h"
#include "chrono_parallel/collision/ChCNarrowphaseMPRUtils.h"
#include "collision/ChCCollisionModel.h"
#include "core/ChMathematics.h"
//
#include "collision/ChCCollisionSystem.h"
#include "collision/ChCModelBullet.h"
#include "collision/bullet/btBulletCollisionCommon.h"
#include "collision/gimpact/GIMPACT/Bullet/btGImpactCollisionAlgorithm.h"

#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"

#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"

using namespace std;
using namespace chrono;
using namespace chrono::collision;

btCollisionConfiguration* bt_collision_configuration;
btCollisionDispatcher* bt_dispatcher;
btBroadphaseInterface* bt_broadphase;
btCollisionWorld* bt_collision_world;

double scene_size = 500;
unsigned int max_objects = 16000;
double envelope = 0.1;

real3 ToReal3(const btVector3& v) {
   return real3(v.x(), v.y(), v.z());
}

btVector3 ToBtVec(const real3& v) {
   return btVector3(v.x, v.y, v.z);
}
void InitBulletCollision() {

   bt_collision_configuration = new btDefaultCollisionConfiguration();
   bt_dispatcher = new btCollisionDispatcher(bt_collision_configuration);

   btScalar sscene_size = (btScalar) scene_size;
   btVector3 worldAabbMin(-sscene_size, -sscene_size, -sscene_size);
   btVector3 worldAabbMax(sscene_size, sscene_size, sscene_size);
   bt_broadphase = new bt32BitAxisSweep3(worldAabbMin, worldAabbMax, max_objects, 0, true);  // true for disabling raycast accelerator

   bt_collision_world = new btCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_configuration);

}

void GetContacts() {
   ChCollisionInfo icontact;

   int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();

   for (int i = 0; i < numManifolds; i++) {

      btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);

      btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
      btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
      contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

      // Execute custom broadphase callback, if any
      bool do_narrow_contactgeneration = true;

      if (do_narrow_contactgeneration) {
         int numContacts = contactManifold->getNumContacts();

         for (int j = 0; j < numContacts; j++) {
            btManifoldPoint& pt = contactManifold->getContactPoint(j);

            btVector3 ptA = pt.getPositionWorldOnA();
            btVector3 ptB = pt.getPositionWorldOnB();

            icontact.vpA.Set(ptA.getX(), ptA.getY(), ptA.getZ());
            icontact.vpB.Set(ptB.getX(), ptB.getY(), ptB.getZ());

            icontact.vN.Set(-pt.m_normalWorldOnB.getX(), -pt.m_normalWorldOnB.getY(), -pt.m_normalWorldOnB.getZ());
            icontact.vN.Normalize();

            double ptdist = pt.getDistance();

            icontact.vpA = icontact.vpA - icontact.vN * envelope;
            icontact.vpB = icontact.vpB + icontact.vN * envelope;

            icontact.distance = ptdist + envelope + envelope;

            icontact.reaction_cache = pt.reactions_cache;

            cout << icontact.distance << endl;
            // Execute some user custom callback, if any

            // Add to contact container
            //mcontactcontainer->AddContact(icontact);

         }

      }
      //you can un-comment out this line, and then all points are removed
      //contactManifold->clearManifold();
   }
}

int main(int argc,
         char* argv[]) {
   InitBulletCollision();

   btCollisionObject* sphere_A = new btCollisionObject();
   btCollisionObject* sphere_B = new btCollisionObject();

   sphere_A->getWorldTransform().setOrigin(btVector3((btScalar) 2, (btScalar) 2.05, (btScalar) 0));
   sphere_B->getWorldTransform().setOrigin(btVector3((btScalar) 2, (btScalar) 0, (btScalar) 0));

   btSphereShape sphere_shape(1+envelope);
   sphere_shape.setMargin(envelope);

   sphere_A->setCollisionShape(&sphere_shape);
   sphere_B->setCollisionShape(&sphere_shape);
   bt_collision_world->addCollisionObject(sphere_A);
   bt_collision_world->addCollisionObject(sphere_B);
   bt_collision_world->performDiscreteCollisionDetection();
   cout << bt_collision_world->getNumCollisionObjects() << endl;

   GetContacts();

   return 0;
}

