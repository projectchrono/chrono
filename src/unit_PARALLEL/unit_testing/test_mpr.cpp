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
//
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
using namespace std;

real3 ToReal3(const btVector3& v){
   return real3(v.x(),v.y(),v.z());
}

btVector3 ToBtVec(const real3& v){
   return btVector3(v.x,v.y,v.z);
}

int main(
   int argc,
   char* argv[]) {

// Support Functions
// =============================================================================
   real3 Dir = normalize(real3(1.123,-2.45,-8));

   {  //Sphere
      real3 R = real3(3.0,1,2);
      real3 answer_a = GetSupportPoint_Sphere(R,Dir);

      btSphereShape shape(R.x);
      shape.setMargin(0);
      real3 answer_b = ToReal3(shape.localGetSupportingVertex(btVector3(Dir.x,Dir.y,Dir.z)));

      StrictEqual(answer_a.x, answer_b.x);
      StrictEqual(answer_a.y, answer_b.y);
      StrictEqual(answer_a.z, answer_b.z);
   }

   {  //Box
       real3 R = real3(3.0,1,2);
       real3 answer_a = GetSupportPoint_Box(R,Dir);

       btBoxShape shape(ToBtVec(R));
       shape.setMargin(0);
       real3 answer_b = ToReal3(shape.localGetSupportingVertex(btVector3(Dir.x,Dir.y,Dir.z)));

       StrictEqual(answer_a.x, answer_b.x);
       StrictEqual(answer_a.y, answer_b.y);
       StrictEqual(answer_a.z, answer_b.z);
    }

   {  //Cylinder
       real3 R = real3(3.0,1,2);
       real3 answer_a = GetSupportPoint_Cylinder(R,Dir);

       btCylinderShape shape(ToBtVec(R));
       shape.setMargin(0);
       real3 answer_b = ToReal3(shape.localGetSupportingVertex(btVector3(Dir.x,Dir.y,Dir.z)));

       StrictEqual(answer_a.x, answer_b.x);
       StrictEqual(answer_a.y, answer_b.y);
       StrictEqual(answer_a.z, answer_b.z);
    }

   {  //Cone
       real3 R = real3(3.0,1,2);
       real3 answer_a = GetSupportPoint_Cone(R,Dir);

       btConeShape shape(R.x,R.y);
       shape.setMargin(0);
       real3 answer_b = ToReal3(shape.localGetSupportingVertex(btVector3(Dir.x,Dir.y,Dir.z)));

       StrictEqual(answer_a.x, answer_b.x);
       StrictEqual(answer_a.y, answer_b.y);
       StrictEqual(answer_a.z, answer_b.z);
    }
      //TODO: Add Ellipsoid test

//Contact tests
// =============================================================================




return 0;
}

