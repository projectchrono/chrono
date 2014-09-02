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
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
using namespace std;
using namespace chrono;
using namespace chrono::collision;
real3 ToReal3(
      const btVector3& v) {
   return real3(v.x(), v.y(), v.z());
}

btVector3 ToBtVec(
      const real3& v) {
   return btVector3(v.x, v.y, v.z);
}

int main(
      int argc,
      char* argv[]) {
   //COMPARE_EPS = 2e-4;
// Support Functions
// =============================================================================
   real3 Dir = normalize(real3(1.123, -2.45, -8));

   {
      std::cout << "Sphere" << endl;
      real3 R = real3(3.0, 1, 2);
      real3 answer_a = GetSupportPoint_Sphere(R, Dir);

      btSphereShape shape(R.x);
      shape.setMargin(0);
      real3 answer_b = R.x * Dir;  //ToReal3(shape.localGetSupportingVertex(btVector3(Dir.x, Dir.y, Dir.z)));

      WeakEqual(answer_a, answer_b);
   }

   {
      std::cout << "Box" << endl;
      real3 R = real3(3.0, 1, 2);
      real3 answer_a = GetSupportPoint_Box(R, Dir);

      btBoxShape shape(ToBtVec(R));
      shape.setMargin(0);
      real3 answer_b = ToReal3(shape.localGetSupportingVertex(btVector3(Dir.x, Dir.y, Dir.z)));

      WeakEqual(answer_a, answer_b);

   }

   {
      std::cout << "Cylinder" << endl;
      real3 R = real3(3.0, 1.0, 3.0);
      real3 answer_a = GetSupportPoint_Cylinder(R, Dir);

      btCylinderShape shape(ToBtVec(R));
      shape.setMargin(0);
      real3 answer_b = ToReal3(shape.localGetSupportingVertex(btVector3(Dir.x, Dir.y, Dir.z)));

      WeakEqual(answer_a, answer_b);

   }

   {
      std::cout << "Cone" << endl;
      real3 R = real3(3.0, 1.0, 3.0);
      real3 answer_a = GetSupportPoint_Cone(R, Dir);

      btConeShape shape(R.x, R.y);
      shape.setMargin(0);
      real3 answer_b = ToReal3(shape.localGetSupportingVertex(btVector3(Dir.x, Dir.y, Dir.z)));

      WeakEqual(answer_a, answer_b);
   }
   //TODO: Add Ellipsoid test

//Contact tests
// =============================================================================
   {
      std::cout << "special two spheres touching perfectly" << endl;
      real3 n;
      real d = 0;
      real3 p1, p2;
      SphereSphere(real3(2, 2, 0), real3(2, 0, 0), real3(1, 0, 0), real3(1, 0, 0), n, d, p1, p2);

      //std::cout << n << p1 << p2 << d << endl;

      WeakEqual(n, real3(0, -1, 0));
      WeakEqual(p1, real3(2, 1, 0));
      WeakEqual(p2, real3(2, 1, 0));
      WeakEqual(d, 0.0);
   }
   {
      std::cout << "special two spheres inter-penetrating" << endl;
      real3 n;
      real d = 0;
      real3 p1, p2;
      SphereSphere(real3(1, 1, 0), real3(2, 0, 0), real3(1, 0, 0), real3(1, 0, 0), n, d, p1, p2);

      //std::cout << n << p1 << p2 << d << endl;
      real3 n_check = real3(sin(CH_C_PI / 4.0), -sin(CH_C_PI / 4.0), 0);
      WeakEqual(n, n_check);
      WeakEqual(p1, real3(1, 1, 0) + n_check * 1);
      WeakEqual(p2, real3(2, 0, 0) - n_check * 1);
      WeakEqual(d, dot(n_check, (real3(2, 0, 0) - n_check * 1) - (real3(1, 1, 0) + n_check * 1)));

   }

   {
      std::cout << "two spheres touching perfectly" << endl;
      real3 p, n(0, 0, 0);
      real d = 0;

      ShapeType A_T = ShapeType::SPHERE;
      real3 A_X = real3(2, 2, 0);
      real3 A_Y = real3(1, 0, 0);
      real3 A_Z = real3(0);
      real4 A_R = real4(1, 0, 0, 0);

      ShapeType B_T = ShapeType::SPHERE;
      real3 B_X = real3(2, 0, 0);
      real3 B_Y = real3(1, 0, 0);
      real3 B_Z = real3(0);
      real4 B_R = real4(1, 0, 0, 0);

      CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, d);
      real3 p1, p2;
      GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, p1, p2);

      //std::cout << n << p1 << p2 << d << endl;
      WeakEqual(n, real3(0, -1, 0));
      WeakEqual(p1, real3(2, 1, 0));
      WeakEqual(p2, real3(2, 1, 0));
      WeakEqual(d, 0.0);

   }
//
   {
      std::cout << "two spheres inter-penetrating" << endl;
      real3 p, n(0, 0, 0);
      real d = 0;

      ShapeType A_T = ShapeType::SPHERE;
      real3 A_X = real3(1, 1, 0);
      real3 A_Y = real3(1, 0, 0);
      real3 A_Z = real3(0);
      real4 A_R = real4(1, 0, 0, 0);

      ShapeType B_T = ShapeType::SPHERE;
      real3 B_X = real3(2, 0, 0);
      real3 B_Y = real3(1, 0, 0);
      real3 B_Z = real3(0);
      real4 B_R = real4(1, 0, 0, 0);

      CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, d);
      real3 p1, p2;
      GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, p1, p2);
      d = dot(n, p2 - p1);
      //std::cout << n << p1 << p2 << d << endl;
      real3 n_check = real3(sin(CH_C_PI / 4.0), -sin(CH_C_PI / 4.0), 0);
      WeakEqual(n, n_check);
      WeakEqual(p1, real3(1, 1, 0) + n_check * 1);
      WeakEqual(p2, real3(2, 0, 0) - n_check * 1);
      WeakEqual(d, dot(n_check, (real3(2, 0, 0) - n_check * 1) - (real3(1, 1, 0) + n_check * 1)));
   }

   {
      std::cout << "two ellipsoids touching perfectly" << endl;
      real3 p, n(0, 0, 0);
      real d = 0;

      ShapeType A_T = ShapeType::ELLIPSOID;
      real3 A_X = real3(2, 2, 0);
      real3 A_Y = real3(1, 1, 1);
      real3 A_Z = real3(0);
      real4 A_R = real4(1, 0, 0, 0);

      ShapeType B_T = ShapeType::ELLIPSOID;
      real3 B_X = real3(2, 0, 0);
      real3 B_Y = real3(1, 1, 1);
      real3 B_Z = real3(0);
      real4 B_R = real4(1, 0, 0, 0);

      CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, d);
      real3 p1, p2;
      GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, p1, p2);

      //std::cout << n << p1 << p2 << d << endl;
      WeakEqual(n, real3(0, -1, 0));
      WeakEqual(p1, real3(2, 1, 0));
      WeakEqual(p2, real3(2, 1, 0));
      WeakEqual(d, 0.0);

   }
//
   {
      std::cout << "two ellipsoids inter-penetrating" << endl;
      real3 p, n(0, 0, 0);
      real d = 0;

      ShapeType A_T = ShapeType::ELLIPSOID;
      real3 A_X = real3(1, 1, 0);
      real3 A_Y = real3(1, 1, 1);
      real3 A_Z = real3(0);
      real4 A_R = real4(1, 0, 0, 0);

      ShapeType B_T = ShapeType::ELLIPSOID;
      real3 B_X = real3(2, 0, 0);
      real3 B_Y = real3(1, 1, 1);
      real3 B_Z = real3(0);
      real4 B_R = real4(1, 0, 0, 0);

      CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, d);
      real3 p1, p2;
      GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, p1, p2);
      d = dot(n, p2 - p1);
      //std::cout << n << p1 << p2 << d << endl;
      real3 n_check = real3(sin(CH_C_PI / 4.0), -sin(CH_C_PI / 4.0), 0);
      WeakEqual(n, n_check);
      WeakEqual(p1, real3(1, 1, 0) + n_check * 1);
      WeakEqual(p2, real3(2, 0, 0) - n_check * 1);
      WeakEqual(d, dot(n_check, (real3(2, 0, 0) - n_check * 1) - (real3(1, 1, 0) + n_check * 1)));
   }

   {
      std::cout << "sphere on box centered" << endl;
      real3 p, n(0, 0, 0);
      real d = 0;

      ShapeType A_T = ShapeType::SPHERE;
      real3 A_X = real3(0, 1.5, 0);
      real3 A_Y = real3(1, 0, 0);
      real3 A_Z = real3(0);
      real4 A_R = real4(1, 0, 0, 0);

      ShapeType B_T = ShapeType::BOX;
      real3 B_X = real3(0, 0, 0);
      real3 B_Y = real3(1, 1, 1);
      real3 B_Z = real3(0);
      real4 B_R = real4(1, 0, 0, 0);

      CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, d);
      real3 p1, p2;
      GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, p1, p2);
      //std::cout << n << p << d << endl << p1 << p2 << endl;

      WeakEqual(n, real3(0, -1, 0));
      WeakEqual(p1, real3(0, 0.5, 0));
      WeakEqual(p2, real3(0, 1, 0));
      WeakEqual(d, -0.5);
   }

   {
      std::cout << "sphere on box offset" << endl;
      real3 p, n(0, 0, 0);
      real d = 0;

      ShapeType A_T = ShapeType::SPHERE;
      real3 A_X = real3(.1, 2, 0);
      real3 A_Y = real3(1, 0, 0);
      real3 A_Z = real3(0);
      real4 A_R = real4(1, 0, 0, 0);

      ShapeType B_T = ShapeType::BOX;
      real3 B_X = real3(0, 0, 0);
      real3 B_Y = real3(1, 1, 1);
      real3 B_Z = real3(0);
      real4 B_R = real4(1, 0, 0, 0);

      if (!CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, d)) {
         std::cout << "No Contact!\n";
      }
      real3 p1, p2;
      GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, p1, p2);
      //std::cout << n << p << d << endl << p1 << p2 << endl;
      WeakEqual(n, real3(0, -1, 0));
      WeakEqual(p1, real3(.1, 1, 0));
      WeakEqual(p2, real3(.1, 1, 0));
      WeakEqual(d, 0);
   }

//   {
//      std::cout << "sphere on box offset and penetrating" << endl;
//      real3 p, n(0, 0, 0);
//      real d = 0;
//
//      ShapeType A_T = ShapeType::SPHERE;
//      real3 A_X = real3(0.629447, 0.045702643641292666, -1.45809);
//      real3 A_Y = real3(0.1, 0, 0);
//      real3 A_Z = real3(0);
//      real4 A_R = real4(1, 0, 0, 0);
//
//      ShapeType B_T = ShapeType::BOX;
//      real3 B_X = real3(0, -.1, 0);
//      real3 B_Y = real3(5, .1, 2);
//      real3 B_Z = real3(0);
//      real4 B_R = real4(1, 0, 0, 0);
//
//      if (!CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, d)) {
//         std::cout << "No Contact!\n";
//      }
//      real3 p1, p2;
//      GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, p1, p2);
//      std::cout << n << p << d << endl << p1 << p2 << endl;
////      WeakEqual(n.x, 0);
////      WeakEqual(n.y, 1);
////      WeakEqual(n.z, 0);
////
////      WeakEqual(p.x, .1);
////      WeakEqual(p.y, 1);
////      WeakEqual(p.z, 0);
////
////      WeakEqual(d, 0);
////
////      WeakEqual(p1.x, .1);
////      WeakEqual(p1.y, 1);
////      WeakEqual(p1.z, 0);
////
////      WeakEqual(p2.x, .1);
////      WeakEqual(p2.y, 1);
////      WeakEqual(p2.z, 0);
//   }
//
//   {
//      std::cout << "sphere on box offset and penetrating Zup" << endl;
//      real3 p, n(0, 0, 0);
//      real d = 0;
//
//      ShapeType A_T = ShapeType::SPHERE;
//      real3 A_X = real3(0.629447, -1.45809, 0.045702643641292666);
//      real3 A_Y = real3(0.1, 0, 0);
//      real3 A_Z = real3(0);
//      real4 A_R = real4(1, 0, 0, 0);
//
//      ShapeType B_T = ShapeType::BOX;
//      real3 B_X = real3(0, 0, -0.1);
//      real3 B_Y = real3(5, 2, 0.1);
//      real3 B_Z = real3(0);
//      real4 B_R = real4(1, 0, 0, 0);
//
//      CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, d);
//      real3 p1, p2;
//      GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, p1, p2);
//      d = dot(n, p2 - p1);
//      std::cout << n << p << d << endl << p1 << p2 << endl;
//      //      WeakEqual(n.x, 0);
//      //      WeakEqual(n.y, 1);
//      //      WeakEqual(n.z, 0);
//      //
//      //      WeakEqual(p.x, .1);
//      //      WeakEqual(p.y, 1);
//      //      WeakEqual(p.z, 0);
//      //
//      //      WeakEqual(d, 0);
//      //
//      //      WeakEqual(p1.x, .1);
//      //      WeakEqual(p1.y, 1);
//      //      WeakEqual(p1.z, 0);
//      //
//      //      WeakEqual(p2.x, .1);
//      //      WeakEqual(p2.y, 1);
//      //      WeakEqual(p2.z, 0);
//   }

//   {
//      std::cout << "box on box offset" << endl;
//      real3 p, n(0, 0, 0);
//      real d = 0;
//
//      ShapeType A_T = ShapeType::BOX;
//      real3 A_X = real3(1, 2, 0);
//      real3 A_Y = real3(1, 1, 1);
//      real3 A_Z = real3(0);
//      real4 A_R = real4(1, 0, 0, 0);
//
//      ShapeType B_T = ShapeType::BOX;
//      real3 B_X = real3(0, 0, 0);
//      real3 B_Y = real3(20, 1, 3);
//      real3 B_Z = real3(0);
//      real4 B_R = real4(1, 0, 0, 0);
//
//      CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, d);
//      real3 p1, p2;
//      GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, p1, p2);
//      std::cout << n << p << d << endl << p1 << p2 << endl;
//      //      WeakEqual(n.x, 0);
//      //      WeakEqual(n.y, 1);
//      //      WeakEqual(n.z, 0);
//      //
//      //      WeakEqual(p.x, .1);
//      //      WeakEqual(p.y, 1);
//      //      WeakEqual(p.z, 0);
//      //
//      //      WeakEqual(d, 0);
//      //
//      //      WeakEqual(p1.x, .1);
//      //      WeakEqual(p1.y, 1);
//      //      WeakEqual(p1.z, 0);
//      //
//      //      WeakEqual(p2.x, .1);
//      //      WeakEqual(p2.y, 1);
//      //      WeakEqual(p2.z, 0);
//   }

   return 0;
}

