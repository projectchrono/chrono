/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BULLET_COLLISION_COMMON_H
#define BULLET_COLLISION_COMMON_H

///Common headerfile includes for Bullet Collision Detection

///Bullet's cbtCollisionWorld and cbtCollisionObject definitions
#include "BulletCollision/CollisionDispatch/cbtCollisionWorld.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObject.h"

///Collision Shapes
#include "BulletCollision/CollisionShapes/cbtBoxShape.h"
#include "BulletCollision/CollisionShapes/cbtSphereShape.h"
#include "BulletCollision/CollisionShapes/cbtCapsuleShape.h"
#include "BulletCollision/CollisionShapes/cbtCylinderShape.h"
#include "BulletCollision/CollisionShapes/cbtCylindricalShellShape.h"  /* ***CHRONO*** */
#include "BulletCollision/CollisionShapes/cbtConeShape.h"
#include "BulletCollision/CollisionShapes/cbtStaticPlaneShape.h"
#include "BulletCollision/CollisionShapes/cbtConvexHullShape.h"
#include "BulletCollision/CollisionShapes/cbtTriangleMesh.h"
#include "BulletCollision/CollisionShapes/cbtConvexTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/cbtBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/cbtScaledBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/cbtTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/cbtTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionShapes/cbtCompoundShape.h"
#include "BulletCollision/CollisionShapes/cbtTetrahedronShape.h"
#include "BulletCollision/CollisionShapes/cbtEmptyShape.h"
#include "BulletCollision/CollisionShapes/cbtMultiSphereShape.h"
#include "BulletCollision/CollisionShapes/cbtUniformScalingShape.h"

///Narrowphase Collision Detector
#include "BulletCollision/CollisionDispatch/cbtSphereSphereCollisionAlgorithm.h"

//#include "BulletCollision/CollisionDispatch/cbtSphereBoxCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/cbtDefaultCollisionConfiguration.h"

///Dispatching and generation of collision pairs (broadphase)
#include "BulletCollision/CollisionDispatch/cbtCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/cbtSimpleBroadphase.h"
#include "BulletCollision/BroadphaseCollision/cbtAxisSweep3.h"
#include "BulletCollision/BroadphaseCollision/cbtDbvtBroadphase.h"

///Math library & Utils
#include "LinearMath/cbtQuaternion.h"
#include "LinearMath/cbtTransform.h"
#include "LinearMath/cbtDefaultMotionState.h"
#include "LinearMath/cbtQuickprof.h"
#include "LinearMath/cbtIDebugDraw.h"
#include "LinearMath/cbtSerializer.h"

#endif  //BULLET_COLLISION_COMMON_H
