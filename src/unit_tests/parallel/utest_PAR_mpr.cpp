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
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// ChronoParallel unit tests for MPR collision detection
//
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono_parallel/collision/ChNarrowphaseUtils.h"
#include "chrono_parallel/collision/ChNarrowphaseMPR.h"

#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/core/ChMathematics.h"

#include "unit_testing.h"
//
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"

using namespace chrono;
using namespace chrono::collision;
using std::cout;
using std::endl;

real envelope = 0;

#ifdef CHRONO_PARALLEL_USE_DOUBLE
const double precision = 5e-7;
#else
const float precision = 1e-6f;
#endif

real3 ToReal3(const btVector3& v) {
    return real3(v.x(), v.y(), v.z());
}

btVector3 ToBtVec(const real3& v) {
    return btVector3((btScalar)v.x, (btScalar)v.y, (btScalar)v.z);
}

// =============================================================================

void test_support_functions() {
    cout << "support_functions" << endl;

    real3 Dir = Normalize(real3(1.123, -2.45, -8));

    {
        cout << "  sphere" << endl;
        real3 R = real3(3.0, 1, 2);
        real3 answer_a = GetSupportPoint_Sphere(R.x, Dir);

        btSphereShape shape((btScalar)R.x);
        shape.setMargin(0);
        real3 answer_b = R.x * Dir;  // ToReal3(shape.localGetSupportingVertex(btVector3(Dir.x, Dir.y, Dir.z)));

        WeakEqual(answer_a, answer_b, precision);
    }

    {
        cout << "  box" << endl;
        real3 R = real3(3.0, 1, 2);
        real3 answer_a = GetSupportPoint_Box(R, Dir);

        btBoxShape shape(ToBtVec(R));
        shape.setMargin(0);
        real3 answer_b = ToReal3(shape.localGetSupportingVertex(btVector3((btScalar)Dir.x, (btScalar)Dir.y, (btScalar)Dir.z)));

        WeakEqual(answer_a, answer_b, precision);
    }

    {
        cout << "  cylinder" << endl;
        real3 R = real3(3.0, 1.0, 3.0);
        real3 answer_a = GetSupportPoint_Cylinder(R, Dir);

        btCylinderShape shape(ToBtVec(R));
        shape.setMargin(0);
        real3 answer_b = ToReal3(shape.localGetSupportingVertex(btVector3((btScalar)Dir.x, (btScalar)Dir.y, (btScalar)Dir.z)));

        WeakEqual(answer_a, answer_b, precision);
    }

    {
        cout << "  cone" << endl;
        real3 R = real3(3.0, 1.0, 3.0);
        real3 answer_a = GetSupportPoint_Cone(R, Dir);

        btConeShape shape((btScalar)R.x, (btScalar)R.y);
        shape.setMargin(0);
        real3 answer_b = ToReal3(shape.localGetSupportingVertex(btVector3((btScalar)Dir.x, (btScalar)Dir.y, (btScalar)Dir.z)));

        WeakEqual(answer_a, answer_b, precision);
    }

    // TODO: Add Ellipsoid test
}

// =============================================================================

void test_sphere_sphere() {
    cout << "sphere_sphere" << endl;

    {
        cout << "  special two spheres touching perfectly" << endl;
        real3 n;
        real d = 0;
        real3 p1, p2;

        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(SPHERE, real3(2, 2, 0), quaternion(1, 0, 0, 0), real3(1, 0, 0));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(SPHERE, real3(2, 0, 0), quaternion(1, 0, 0, 0), real3(1, 0, 0));

        MPRSphereSphere(shapeA, shapeB, n, d, p1, p2);

         cout << n << p1 << p2 << d << endl;

        WeakEqual(n, real3(0, -1, 0), precision);
        WeakEqual(p1, real3(2, 1, 0), precision);
        WeakEqual(p2, real3(2, 1, 0), precision);
        WeakEqual(d, 0.0);

        delete shapeA;
        delete shapeB;
    }

    {
        cout << "  special two spheres inter-penetrating" << endl;
        real3 n;
        real d = 0;
        real3 p1, p2;

        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(SPHERE, real3(1, 1, 0), quaternion(1, 0, 0, 0), real3(1, 0, 0));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(SPHERE, real3(0, 0, 0), quaternion(1, 0, 0, 0), real3(1, 0, 0));

        MPRSphereSphere(shapeA, shapeB, n, d, p1, p2);

        // cout << n << p1 << p2 << d << endl;
        real3 n_check = real3(-sin(CH_C_PI / 4.0), -sin(CH_C_PI / 4.0), 0);
        WeakEqual(n, n_check, precision);
        WeakEqual(p1, real3(1, 1, 0) + n_check * 1, precision);
        WeakEqual(p2, real3(0, 0, 0) - n_check * 1, precision);
        WeakEqual(d, Dot(n_check, (real3(0, 0, 0) - n_check * 1) - (real3(1, 1, 0) + n_check * 1)), precision);
        delete shapeA;
        delete shapeB;
    }

    {
        cout << "  two spheres touching perfectly" << endl;
        real3 p, n(0, 0, 0);
        real d = 0;

        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(SPHERE, real3(2, 2, 0), quaternion(1, 0, 0, 0), real3(1, 0, 0));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(SPHERE, real3(2, 0, 0), quaternion(1, 0, 0, 0), real3(1, 0, 0));

        MPRContact(shapeA, shapeB, envelope, n, p, d);
        real3 p1, p2;
        MPRGetPoints(shapeA, shapeB, envelope, n, p, p1, p2);

        // cout << n << p1 << p2 << d << endl;
        WeakEqual(n, real3(0, -1, 0), precision);
        WeakEqual(p1, real3(2, 1, 0), precision);
        WeakEqual(p2, real3(2, 1, 0), precision);
        WeakEqual(d, 0.0, precision);
        delete shapeA;
        delete shapeB;
    }

    {
        cout << "  two spheres inter-penetrating" << endl;
        real3 p, n(0, 0, 0);
        real d = 0;
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(SPHERE, real3(1, 1, 0), quaternion(1, 0, 0, 0), real3(1, 0, 0));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(SPHERE, real3(2, 0, 0), quaternion(1, 0, 0, 0), real3(1, 0, 0));

        MPRContact(shapeA, shapeB, envelope, n, p, d);
        real3 p1, p2;
        MPRGetPoints(shapeA, shapeB, envelope, n, p, p1, p2);
        d = Dot(n, p2 - p1);
        // cout << n << p1 << p2 << d << endl;
        real3 n_check = real3(sin(CH_C_PI / 4.0), -sin(CH_C_PI / 4.0), 0);
        WeakEqual(n, n_check, precision);
        WeakEqual(p1, real3(1, 1, 0) + n_check * 1, precision);
        WeakEqual(p2, real3(2, 0, 0) - n_check * 1, precision);
        WeakEqual(d, Dot(n_check, (real3(2, 0, 0) - n_check * 1) - (real3(1, 1, 0) + n_check * 1)), precision);
        delete shapeA;
        delete shapeB;
    }
}

// =============================================================================

void test_ellipsoid_ellipsoid() {
    cout << "ellipsoid_ellipsoid" << endl;

    {
        cout << "  two ellipsoids touching perfectly" << endl;
        real3 p, n(0, 0, 0);
        real d = 0;

        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ELLIPSOID, real3(2, 2, 0), quaternion(1, 0, 0, 0), real3(1, 1, 1));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ELLIPSOID, real3(2, 0, 0), quaternion(1, 0, 0, 0), real3(1, 1, 1));

        MPRContact(shapeA, shapeB, envelope, n, p, d);
        real3 p1, p2;
        MPRGetPoints(shapeA, shapeB, envelope, n, p, p1, p2);

        // cout << n << p1 << p2 << d << endl;
        WeakEqual(n, real3(0, -1, 0), precision);
        WeakEqual(p1, real3(2, 1, 0), precision);
        WeakEqual(p2, real3(2, 1, 0), precision);
        WeakEqual(d, 0.0, precision);
        delete shapeA;
        delete shapeB;
    }

    {
        cout << "  two ellipsoids inter-penetrating" << endl;
        real3 p, n(0, 0, 0);
        real d = 0;

        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ELLIPSOID, real3(1, 1, 0), quaternion(1, 0, 0, 0), real3(1, 1, 1));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ELLIPSOID, real3(2, 0, 0), quaternion(1, 0, 0, 0), real3(1, 1, 1));

        MPRContact(shapeA, shapeB, envelope, n, p, d);
        real3 p1, p2;
        MPRGetPoints(shapeA, shapeB, envelope, n, p, p1, p2);
        d = Dot(n, p2 - p1);
        // cout << n << p1 << p2 << d << endl;

        // sResults res;
        // GJKCollide(shapeA, shapeB, res);

        // cout << res.normal << res.witnesses[0] << res.witnesses[1] << res.distance << " " << res.status << endl;
        // cout << n << p1 << p2 << d << endl;

        real3 n_check = real3(sin(CH_C_PI / 4.0), -sin(CH_C_PI / 4.0), 0);
        WeakEqual(n, n_check, precision);
        WeakEqual(p1, real3(1, 1, 0) + n_check * 1, precision);
        WeakEqual(p2, real3(2, 0, 0) - n_check * 1, precision);
        WeakEqual(d, Dot(n_check, (real3(2, 0, 0) - n_check * 1) - (real3(1, 1, 0) + n_check * 1)), precision);

        delete shapeA;
        delete shapeB;
    }
}

// =============================================================================

void test_sphere_box() {
    cout << "sphere_box" << endl;

    {
        cout << "  sphere on box centered" << endl;
        real3 p, n(0, 0, 0);
        real d = 0;
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(SPHERE, real3(0, 1.5, 0), quaternion(1, 0, 0, 0), real3(1, 0, 0));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(BOX, real3(0, 0, 0), quaternion(1, 0, 0, 0), real3(1, 1, 1));

        MPRContact(shapeA, shapeB, envelope, n, p, d);
        real3 p1, p2;
        MPRGetPoints(shapeA, shapeB, envelope, n, p, p1, p2);
        // cout << n << p << d << endl << p1 << p2 << endl;

        WeakEqual(n, real3(0, -1, 0), precision);
        WeakEqual(p1, real3(0, 0.5, 0), precision);
        WeakEqual(p2, real3(0, 1, 0), precision);
        WeakEqual(d, -0.5, precision);
        delete shapeA;
        delete shapeB;
    }

    {
        cout << "  sphere on box offset" << endl;
        real3 p, n(0, 0, 0);
        real d = 0;
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(SPHERE, real3(.1, 2, 0), quaternion(1, 0, 0, 0), real3(1, 0, 0));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(BOX, real3(0, 0, 0), quaternion(1, 0, 0, 0), real3(1, 1, 1));

        if (!MPRContact(shapeA, shapeB, envelope, n, p, d)) {
            cout << "No Contact!\n";
        }
        real3 p1, p2;
        MPRGetPoints(shapeA, shapeB, envelope, n, p, p1, p2);
        // cout << n << p << d << endl << p1 << p2 << endl;
        WeakEqual(n, real3(0, -1, 0), precision);
        WeakEqual(p1, real3(.1, 1, 0), precision);
        WeakEqual(p2, real3(.1, 1, 0), precision);
        WeakEqual(d, 0, precision);
        delete shapeA;
        delete shapeB;
    }

    /*
     {
     cout << "  sphere on box offset and penetrating" << endl;
     real3 p, n(0, 0, 0);
     real d = 0;

     ShapeType A_T = ShapeType::SPHERE;
     real3 A_X = real3(0.629447, 0.045702643641292666, -1.45809);
     real3 A_Y = real3(0.1, 0, 0);
     real3 A_Z = real3(0);
     real4 A_R = real4(1, 0, 0, 0);

     ShapeType B_T = ShapeType::BOX;
     real3 B_X = real3(0, -.1, 0);
     real3 B_Y = real3(5, .1, 2);
     real3 B_Z = real3(0);
     real4 B_R = real4(1, 0, 0, 0);

     if (!CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, d)) {
     cout << "No Contact!\n";
     }
     real3 p1, p2;
     GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, p1, p2);
     cout << n << p << d << endl << p1 << p2 << endl;
     //      WeakEqual(n.x, 0);
     //      WeakEqual(n.y, 1);
     //      WeakEqual(n.z, 0);
     //
     //      WeakEqual(p.x, .1);
     //      WeakEqual(p.y, 1);
     //      WeakEqual(p.z, 0);
     //
     //      WeakEqual(d, 0);
     //
     //      WeakEqual(p1.x, .1);
     //      WeakEqual(p1.y, 1);
     //      WeakEqual(p1.z, 0);
     //
     //      WeakEqual(p2.x, .1);
     //      WeakEqual(p2.y, 1);
     //      WeakEqual(p2.z, 0);
     }
     */

    /*
     {
     cout << "  sphere on box offset and penetrating Zup" << endl;
     real3 p, n(0, 0, 0);
     real d = 0;

     ShapeType A_T = ShapeType::SPHERE;
     real3 A_X = real3(0.629447, -1.45809, 0.045702643641292666);
     real3 A_Y = real3(0.1, 0, 0);
     real3 A_Z = real3(0);
     real4 A_R = real4(1, 0, 0, 0);

     ShapeType B_T = ShapeType::BOX;
     real3 B_X = real3(0, 0, -0.1);
     real3 B_Y = real3(5, 2, 0.1);
     real3 B_Z = real3(0);
     real4 B_R = real4(1, 0, 0, 0);

     CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, d);
     real3 p1, p2;
     GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, p1, p2);
     d = Dot(n, p2 - p1);
     cout << n << p << d << endl << p1 << p2 << endl;
     //      WeakEqual(n.x, 0);
     //      WeakEqual(n.y, 1);
     //      WeakEqual(n.z, 0);
     //
     //      WeakEqual(p.x, .1);
     //      WeakEqual(p.y, 1);
     //      WeakEqual(p.z, 0);
     //
     //      WeakEqual(d, 0);
     //
     //      WeakEqual(p1.x, .1);
     //      WeakEqual(p1.y, 1);
     //      WeakEqual(p1.z, 0);
     //
     //      WeakEqual(p2.x, .1);
     //      WeakEqual(p2.y, 1);
     //      WeakEqual(p2.z, 0);
     }
     */
}

// =============================================================================

void test_box_box() {
    cout << "box_box" << endl;

    /*
     {
     cout << "  box on box offset" << endl;
     real3 p, n(0, 0, 0);
     real d = 0;

     ShapeType A_T = ShapeType::BOX;
     real3 A_X = real3(1, 2, 0);
     real3 A_Y = real3(1, 1, 1);
     real3 A_Z = real3(0);
     real4 A_R = real4(1, 0, 0, 0);

     ShapeType B_T = ShapeType::BOX;
     real3 B_X = real3(0, 0, 0);
     real3 B_Y = real3(20, 1, 3);
     real3 B_Z = real3(0);
     real4 B_R = real4(1, 0, 0, 0);

     CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, d);
     real3 p1, p2;
     GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, p1, p2);
     cout << n << p << d << endl << p1 << p2 << endl;
     //      WeakEqual(n.x, 0);
     //      WeakEqual(n.y, 1);
     //      WeakEqual(n.z, 0);
     //
     //      WeakEqual(p.x, .1);
     //      WeakEqual(p.y, 1);
     //      WeakEqual(p.z, 0);
     //
     //      WeakEqual(d, 0);
     //
     //      WeakEqual(p1.x, .1);
     //      WeakEqual(p1.y, 1);
     //      WeakEqual(p1.z, 0);
     //
     //      WeakEqual(p2.x, .1);
     //      WeakEqual(p2.y, 1);
     //      WeakEqual(p2.z, 0);
     }
     */
}

// =============================================================================

void test_cylinder_sphere() {
    cout << "cylinder_sphere" << endl;

    real c_rad = 2.0;
    real c_hlen = 1.5;
    real s_rad = 1.0;

    // Cylinder position and orientation fixed for all tests.
    // Aligned with X axis and shifted by its half-length in the X direction.
    real3 c_pos(c_hlen, 0, 0);
    quaternion c_rot = ToQuaternion(Q_from_AngAxis(CH_C_PI_2, ChVector<>(0, 0, 1)));

    real3 norm;
    real depth;
    real3 pt;
    real3 pt1;
    real3 pt2;

    real oosqrt2 = sqrt(0.5);  // 1/sqrt(2)

    {
        cout << "  sphere center inside cylinder" << endl;
        real3 s_pos(2.5, 1.5, 0);
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(CYLINDER, c_pos, c_rot, real3(c_rad, c_hlen, c_rad));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        delete shapeA;
        delete shapeB;
        //// TODO:  WHAT IS EXPECTED HERE?
        /*
         if (res) {
         cout << "    test failed" << endl;
         exit(1);
         }
         */
    }

    {
        cout << "  cap interaction (separated)" << endl;
        real3 s_pos(4.5, 1.5, 0);

        ConvexShapeCustom* shapeA = new ConvexShapeCustom(CYLINDER, c_pos, c_rot, real3(c_rad, c_hlen, c_rad));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        delete shapeA;
        delete shapeB;
        if (res) {
            cout << "    test failed" << endl;
            exit(1);
        }
    }

    {
        cout << "  cap interaction (penetrated)" << endl;
        real3 s_pos(3.75, 1.5, 0);

        ConvexShapeCustom* shapeA = new ConvexShapeCustom(CYLINDER, c_pos, c_rot, real3(c_rad, c_hlen, c_rad));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        MPRGetPoints(shapeA, shapeB, envelope, norm, pt, pt1, pt2);
        depth = Dot(norm, pt2 - pt1);

        if (!res) {
            cout << "    test failed" << endl;
            exit(1);
        }
        WeakEqual(norm, real3(1, 0, 0), precision);
        WeakEqual(depth, -0.25, precision);
        WeakEqual(pt1, real3(3, 1.5, 0), precision);
        WeakEqual(pt2, real3(2.75, 1.5, 0), precision);
        delete shapeA;
        delete shapeB;
    }

    {
        cout << "  side interaction (separated)" << endl;
        real3 s_pos(2.5, 3.5, 0);
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(CYLINDER, c_pos, c_rot, real3(c_rad, c_hlen, c_rad));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        delete shapeA;
        delete shapeB;
        if (res) {
            cout << "    test failed" << endl;
            exit(1);
        }
    }

    {
        cout << "  side interaction (penetrated)" << endl;
        real3 s_pos(2.5, 2.5, 0);
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(CYLINDER, c_pos, c_rot, real3(c_rad, c_hlen, c_rad));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        MPRGetPoints(shapeA, shapeB, envelope, norm, pt, pt1, pt2);
        depth = Dot(norm, pt2 - pt1);

        if (!res) {
            cout << "    test failed" << endl;
            exit(1);
        }
        WeakEqual(norm, real3(0, 1, 0), precision);
        WeakEqual(depth, -0.5, precision);
        WeakEqual(pt1, real3(2.5, 2.0, 0), precision);
        WeakEqual(pt2, real3(2.5, 1.5, 0), precision);

        delete shapeA;
        delete shapeB;
    }

    {
        cout << "  edge interaction (separated)" << endl;
        real3 s_pos(4, 3, 0);
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(CYLINDER, c_pos, c_rot, real3(c_rad, c_hlen, c_rad));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));
        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        if (res) {
            cout << "    test failed" << endl;
            exit(1);
        }
        delete shapeA;
        delete shapeB;
    }

    //// TODO:  FIGURE OUT WHY THE FOLLOWING TEST FAILS!!!
    /*
     {
     cout << "  edge interaction (penetrated)" << endl;
     real3 s_pos(3.5, 0, 2.5);
     c_hlen = 3.0;
     ConvexShape shapeA, shapeB;
     c_pos = real3(0, 0, 0);
     shapeA.type = ShapeType::CYLINDER;
     shapeA.A = c_pos;
     shapeA.B = real3(c_rad, c_hlen, c_rad);
     shapeA.C = real3(0);
     shapeA.R = c_rot;

     shapeB.type = ShapeType::ELLIPSOID;
     shapeB.A = s_pos;
     shapeB.B = real3(s_rad, s_rad, s_rad);
     shapeB.C = real3(0);
     shapeB.R = quaternion(.5, 0, 0, 0);
     bool res = MPRContact(shapeA, shapeB, norm, pt, depth);
     MPRGetPoints(shapeA, shapeB, norm, pt, pt1, pt2);
     depth = Dot(norm, pt2 - pt1);

     //sResults sres;
     //GJKCollide(shapeB, shapeA, sres);

     //cout << sres.normal << sres.witnesses[0] << sres.witnesses[1] << sres.distance << " " << sres.status << endl;

     if (!res) {
     cout << "    test failed" << endl;
     exit(1);
     }
     cout << "    norm" << endl;
     WeakEqual(norm, real3(oosqrt2, oosqrt2, 0), precision);
     cout << "    depth" << endl;
     WeakEqual(depth, -1 + oosqrt2, precision);
     cout << "    pt1" << endl;
     WeakEqual(pt1, real3(3.0, 2.0, 0), precision);
     cout << "    pt2" << endl;
     WeakEqual(pt2, real3(3.5 - oosqrt2, 2.5 - oosqrt2, 0), precision);
     }
     */
}

// =============================================================================

void test_roundedcyl_sphere() {
    cout << "roundedcyl_sphere" << endl;

    real c_rad = 2.0;   // radius of skeleton cylinder
    real c_hlen = 1.5;  // half-length of skeleton cylinder
    real c_srad = 0.1;  // radius of sweeping sphere

    real s_rad = 1.0;  // sphere radius

    // Rounded cylinder position and orientation fixed for all tests.
    // Aligned with X axis and shifted by its half-length in the X direction.
    real3 c_pos(c_hlen, 0, 0);
    quaternion c_rot = ToQuaternion(Q_from_AngAxis(CH_C_PI_2, ChVector<>(0, 0, 1)));

    real3 norm;
    real depth;
    real3 pt;
    real3 pt1;
    real3 pt2;

    real oosqrt2 = sqrt(0.5);  // 1/sqrt(2)

    {
        cout << "  sphere center inside cylinder" << endl;
        real3 s_pos(2.5, 1.5, 0);
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ROUNDEDCYL, c_pos, c_rot, real3(c_rad, c_hlen, c_rad), c_srad);
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        delete shapeA;
        delete shapeB;
        //// TODO: WHAT IS EXPECTED HERE?
        /*
         if (res) {
         cout << "    test failed" << endl;
         exit(1);
         }
         */
    }

    {
        cout << "  cap interaction (separated)" << endl;
        real3 s_pos(4.5, 1.5, 0);
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ROUNDEDCYL, c_pos, c_rot, real3(c_rad, c_hlen, c_rad), c_srad);
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);

        if (res) {
            cout << "    test failed" << endl;
            exit(1);
        }
        delete shapeA;
        delete shapeB;
    }

    {
        cout << "  cap interaction (penetrated)" << endl;
        real3 s_pos(3.75, 1.5, 0);
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ROUNDEDCYL, c_pos, c_rot, real3(c_rad, c_hlen, c_rad), c_srad);
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));
        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        MPRGetPoints(shapeA, shapeB, envelope, norm, pt, pt1, pt2);
        depth = Dot(norm, pt2 - pt1);

        if (!res) {
            cout << "    test failed" << endl;
            exit(1);
        }
        WeakEqual(norm, real3(1, 0, 0), precision);
        WeakEqual(depth, -0.35, precision);
        WeakEqual(pt1, real3(3.1, 1.5, 0), precision);
        WeakEqual(pt2, real3(2.75, 1.5, 0), precision);
        delete shapeA;
        delete shapeB;
    }

    {
        cout << "  side interaction (separated)" << endl;
        real3 s_pos(2.5, 3.5, 0);
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ROUNDEDCYL, c_pos, c_rot, real3(c_rad, c_hlen, c_rad), c_srad);
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        if (res) {
            cout << "    test failed" << endl;
            exit(1);
        }
        delete shapeA;
        delete shapeB;
    }

    {
        cout << "  side interaction (penetrated)" << endl;
        real3 s_pos(2.5, 2.5, 0);
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ROUNDEDCYL, c_pos, c_rot, real3(c_rad, c_hlen, c_rad), c_srad);
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));
        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        MPRGetPoints(shapeA, shapeB, envelope, norm, pt, pt1, pt2);
        depth = Dot(norm, pt2 - pt1);

        if (!res) {
            cout << "    test failed" << endl;
            exit(1);
        }
        WeakEqual(norm, real3(0, 1, 0), precision);
        WeakEqual(depth, -0.6, precision);
        WeakEqual(pt1, real3(2.5, 2.1, 0), precision);
        WeakEqual(pt2, real3(2.5, 1.5, 0), precision);
        delete shapeA;
        delete shapeB;
    }

    {
        cout << "  edge interaction (separated)" << endl;
        real3 s_pos(4, 3, 0);
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ROUNDEDCYL, c_pos, c_rot, real3(c_rad, c_hlen, c_rad), c_srad);
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));
        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        if (res) {
            cout << "    test failed" << endl;
            exit(1);
        }
        delete shapeA;
        delete shapeB;
    }

    //// TODO:  FIGURE OUT WHY THE FOLLOWING TEST FAILS!!!
    /*
     {
     cout << "  edge interaction (penetrated)" << endl;
     real3 s_pos(3.5, 2.5, 0);
     ConvexShape shapeA, shapeB;
     shapeA.type = ShapeType::ROUNDEDCYL;
     shapeA.A = c_pos;
     shapeA.B = real3(c_rad, c_hlen, c_rad);
     shapeA.C = real3(c_srad, 0, 0);
     shapeA.R = c_rot;

     shapeB.type = ShapeType::SPHERE;
     shapeB.A = s_pos;
     shapeB.B = real3(s_rad, 0, 0);
     shapeB.C = real3(0);
     shapeB.R = quaternion(1, 0, 0, 0);
     bool res = MPRContact(shapeA, shapeB, norm, pt, depth);
     MPRGetPoints(shapeA, shapeB, norm, pt, pt1, pt2);
     depth = Dot(norm, pt2 - pt1);

     if (!res) {
     cout << "    test failed" << endl;
     exit(1);
     }
     WeakEqual(norm, real3(oosqrt2, oosqrt2, 0), precision);
     WeakEqual(depth, -1.1 + oosqrt2, precision);
     WeakEqual(pt1, real3(3.0 + 0.1 * oosqrt2, 2.0 + 0.1 * oosqrt2, 0), precision);
     WeakEqual(pt2, real3(3.5 - oosqrt2, 2.5 - oosqrt2, 0), precision);
     }
     */
}

// =============================================================================

int main(int argc, char* argv[]) {
    // COMPARE_EPS = 2e-4;

    // Support functions
    // -----------------

    test_support_functions();

    // Contact tests
    // -------------

    test_sphere_sphere();
    test_ellipsoid_ellipsoid();
    test_sphere_box();

    //// TODO: the following test is not yet implemented
    // test_box_box();

    //// TODO: Check the cases that fail in the following two tests
    //// (currently commented out)
    test_cylinder_sphere();
    test_roundedcyl_sphere();

    return 0;
}
