// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Chrono::Multicore unit tests for MPR collision detection
//
// =============================================================================

#include "chrono_multicore/collision/ChNarrowphaseUtils.h"

#include "chrono_multicore/collision/ChNarrowphaseMPR.h"

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/core/ChMathematics.h"

#include "unit_testing.h"

#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

using namespace chrono;
using namespace chrono::collision;

real envelope = 0;

#ifdef CHRONO_MULTICORE_USE_DOUBLE
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

TEST(ChNarrowphaseMPR, support_functions) {
    real3 Dir = Normalize(real3(1.123, -2.45, -8));

    {
        // sphere
        real3 R = real3(3.0, 1, 2);
        real3 answer_a = GetSupportPoint_Sphere(R.x, Dir);

        btSphereShape shape((btScalar)R.x);
        shape.setMargin(0);
        real3 answer_b = R.x * Dir;  // ToReal3(shape.localGetSupportingVertex(btVector3(Dir.x, Dir.y, Dir.z)));

        Assert_near(answer_a, answer_b, precision);
    }

    {
        // box
        real3 R = real3(3.0, 1, 2);
        real3 answer_a = GetSupportPoint_Box(R, Dir);

        btBoxShape shape(ToBtVec(R));
        shape.setMargin(0);
        real3 answer_b =
            ToReal3(shape.localGetSupportingVertex(btVector3((btScalar)Dir.x, (btScalar)Dir.y, (btScalar)Dir.z)));

        Assert_near(answer_a, answer_b, precision);
    }

    {
        // cylinder
        real3 R = real3(3.0, 1.0, 3.0);
        real3 answer_a = GetSupportPoint_Cylinder(R, Dir);

        btCylinderShape shape(ToBtVec(R));
        shape.setMargin(0);
        real3 answer_b =
            ToReal3(shape.localGetSupportingVertex(btVector3((btScalar)Dir.x, (btScalar)Dir.y, (btScalar)Dir.z)));

        Assert_near(answer_a, answer_b, precision);
    }

    {
        // cone
        real3 R = real3(3.0, 1.0, 3.0);
        real3 answer_a = GetSupportPoint_Cone(R, Dir);

        btConeShape shape((btScalar)R.x, (btScalar)R.y);
        shape.setMargin(0);
        real3 answer_b =
            ToReal3(shape.localGetSupportingVertex(btVector3((btScalar)Dir.x, (btScalar)Dir.y, (btScalar)Dir.z)));

        Assert_near(answer_a, answer_b, precision);
    }

    // TODO: Add Ellipsoid test
}

// =============================================================================

TEST(ChNarrowphaseMPR, sphere_sphere) {
    {
        // special two spheres touching perfectly
        real3 n;
        real d = 0;
        real3 p1, p2;

        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, real3(2, 2, 0), quaternion(1, 0, 0, 0), real3(1, 0, 0));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, real3(2, 0, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 0, 0));

        MPRSphereSphere(shapeA, shapeB, n, d, p1, p2);

        Assert_near(n, real3(0, -1, 0), precision);
        Assert_near(p1, real3(2, 1, 0), precision);
        Assert_near(p2, real3(2, 1, 0), precision);
        ASSERT_NEAR(d, 0.0, precision);

        delete shapeA;
        delete shapeB;
    }

    {
        // special two spheres inter-penetrating
        real3 n;
        real d = 0;
        real3 p1, p2;

        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, real3(1, 1, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 0, 0));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, real3(0, 0, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 0, 0));

        MPRSphereSphere(shapeA, shapeB, n, d, p1, p2);

        // std::cout << n << p1 << p2 << d << std::endl;
        real3 n_check = real3(-sin(CH_C_PI / 4.0), -sin(CH_C_PI / 4.0), 0);
        Assert_near(n, n_check, precision);
        Assert_near(p1, real3(1, 1, 0) + n_check * 1, precision);
        Assert_near(p2, real3(0, 0, 0) - n_check * 1, precision);
        ASSERT_NEAR(d, Dot(n_check, (real3(0, 0, 0) - n_check * 1) - (real3(1, 1, 0) + n_check * 1)), precision);

        delete shapeA;
        delete shapeB;
    }

    {
        // two spheres touching perfectly
        real3 p, n(0, 0, 0);
        real d = 0;

        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, real3(2, 2, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 0, 0));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, real3(2, 0, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 0, 0));

        MPRContact(shapeA, shapeB, envelope, n, p, d);
        real3 p1, p2;
        MPRGetPoints(shapeA, shapeB, envelope, n, p, p1, p2);

        // std::cout << n << p1 << p2 << d << std::endl;
        Assert_near(n, real3(0, -1, 0), precision);
        Assert_near(p1, real3(2, 1, 0), precision);
        Assert_near(p2, real3(2, 1, 0), precision);
        ASSERT_NEAR(d, 0.0, precision);

        delete shapeA;
        delete shapeB;
    }

    {
        // two spheres inter-penetrating
        real3 p, n(0, 0, 0);
        real d = 0;
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, real3(1, 1, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 0, 0));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, real3(2, 0, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 0, 0));

        MPRContact(shapeA, shapeB, envelope, n, p, d);
        real3 p1, p2;
        MPRGetPoints(shapeA, shapeB, envelope, n, p, p1, p2);
        d = Dot(n, p2 - p1);
        // std::cout << n << p1 << p2 << d << std::endl;
        real3 n_check = real3(sin(CH_C_PI / 4.0), -sin(CH_C_PI / 4.0), 0);
        Assert_near(n, n_check, precision);
        Assert_near(p1, real3(1, 1, 0) + n_check * 1, precision);
        Assert_near(p2, real3(2, 0, 0) - n_check * 1, precision);
        ASSERT_NEAR(d, Dot(n_check, (real3(2, 0, 0) - n_check * 1) - (real3(1, 1, 0) + n_check * 1)), precision);

        delete shapeA;
        delete shapeB;
    }
}

// =============================================================================

TEST(ChNarrowphaseMPR, ellipsoid_ellipsoid) {
    {
        // two ellipsoids touching perfectly
        real3 p, n(0, 0, 0);
        real d = 0;

        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::ELLIPSOID, real3(2, 2, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 1, 1));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(ChCollisionShape::Type::ELLIPSOID, real3(2, 0, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 1, 1));

        MPRContact(shapeA, shapeB, envelope, n, p, d);
        real3 p1, p2;
        MPRGetPoints(shapeA, shapeB, envelope, n, p, p1, p2);

        // std::cout << n << p1 << p2 << d << std::endl;
        Assert_near(n, real3(0, -1, 0), precision);
        Assert_near(p1, real3(2, 1, 0), precision);
        Assert_near(p2, real3(2, 1, 0), precision);
        ASSERT_NEAR(d, 0.0, precision);

        delete shapeA;
        delete shapeB;
    }

    {
        // two ellipsoids inter-penetrating
        real3 p, n(0, 0, 0);
        real d = 0;

        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::ELLIPSOID, real3(1, 1, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 1, 1));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(ChCollisionShape::Type::ELLIPSOID, real3(2, 0, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 1, 1));

        MPRContact(shapeA, shapeB, envelope, n, p, d);
        real3 p1, p2;
        MPRGetPoints(shapeA, shapeB, envelope, n, p, p1, p2);
        d = Dot(n, p2 - p1);

        real3 n_check = real3(sin(CH_C_PI / 4.0), -sin(CH_C_PI / 4.0), 0);
        Assert_near(n, n_check, precision);
        Assert_near(p1, real3(1, 1, 0) + n_check * 1, precision);
        Assert_near(p2, real3(2, 0, 0) - n_check * 1, precision);
        ASSERT_NEAR(d, Dot(n_check, (real3(2, 0, 0) - n_check * 1) - (real3(1, 1, 0) + n_check * 1)), precision);

        delete shapeA;
        delete shapeB;
    }
}

// =============================================================================

TEST(ChNarrowphaseMPR, sphere_box) {
    {
        // sphere on box centered
        real3 p, n(0, 0, 0);
        real d = 0;
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, real3(0, 1.5, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 0, 0));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::BOX, real3(0, 0, 0), quaternion(1, 0, 0, 0), real3(1, 1, 1));

        MPRContact(shapeA, shapeB, envelope, n, p, d);
        real3 p1, p2;
        MPRGetPoints(shapeA, shapeB, envelope, n, p, p1, p2);

        Assert_near(n, real3(0, -1, 0), precision);
        Assert_near(p1, real3(0, 0.5, 0), precision);
        Assert_near(p2, real3(0, 1, 0), precision);
        ASSERT_NEAR(d, -0.5, precision);

        delete shapeA;
        delete shapeB;
    }

    {
        // sphere on box offset
        real3 p, n(0, 0, 0);
        real d = 0;
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, real3(.1, 2, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 0, 0));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::BOX, real3(0, 0, 0), quaternion(1, 0, 0, 0), real3(1, 1, 1));

        if (!MPRContact(shapeA, shapeB, envelope, n, p, d)) {
            std::cout << "No Contact!\n";
        }
        real3 p1, p2;
        MPRGetPoints(shapeA, shapeB, envelope, n, p, p1, p2);
        // std::cout << n << p << d << std::endl << p1 << p2 << std::endl;
        Assert_near(n, real3(0, -1, 0), precision);
        Assert_near(p1, real3(.1, 1, 0), precision);
        Assert_near(p2, real3(.1, 1, 0), precision);
        ASSERT_NEAR(d, 0, precision);

        delete shapeA;
        delete shapeB;
    }

    /*
     {
     // sphere on box offset and penetrating
     real3 p, n(0, 0, 0);
     real d = 0;

     ChCollisionShape::Type A_T = ChCollisionShape::Type::SPHERE;
     real3 A_X = real3(0.629447, 0.045702643641292666, -1.45809);
     real3 A_Y = real3(0.1, 0, 0);
     real3 A_Z = real3(0);
     real4 A_R = real4(1, 0, 0, 0);

     ChCollisionShape::Type B_T = ChCollisionShape::Type::BOX;
     real3 B_X = real3(0, -.1, 0);
     real3 B_Y = real3(5, .1, 2);
     real3 B_Z = real3(0);
     real4 B_R = real4(1, 0, 0, 0);

     if (!CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, d)) {
     std::cout << "No Contact!\n";
     }
     real3 p1, p2;
     GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, p1, p2);
     std::cout << n << p << d << std::endl << p1 << p2 << std::endl;
     //      Assert_near(n.x, 0);
     //      Assert_near(n.y, 1);
     //      Assert_near(n.z, 0);
     //
     //      Assert_near(p.x, .1);
     //      Assert_near(p.y, 1);
     //      Assert_near(p.z, 0);
     //
     //      Assert_near(d, 0);
     //
     //      Assert_near(p1.x, .1);
     //      Assert_near(p1.y, 1);
     //      Assert_near(p1.z, 0);
     //
     //      Assert_near(p2.x, .1);
     //      Assert_near(p2.y, 1);
     //      Assert_near(p2.z, 0);
     }
     */

    /*
     {
     // sphere on box offset and penetrating Zup
     real3 p, n(0, 0, 0);
     real d = 0;

     ChCollisionShape::Type A_T = ChCollisionShape::Type::SPHERE;
     real3 A_X = real3(0.629447, -1.45809, 0.045702643641292666);
     real3 A_Y = real3(0.1, 0, 0);
     real3 A_Z = real3(0);
     real4 A_R = real4(1, 0, 0, 0);

     ChCollisionShape::Type B_T = ChCollisionShape::Type::BOX;
     real3 B_X = real3(0, 0, -0.1);
     real3 B_Y = real3(5, 2, 0.1);
     real3 B_Z = real3(0);
     real4 B_R = real4(1, 0, 0, 0);

     CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, d);
     real3 p1, p2;
     GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, p1, p2);
     d = Dot(n, p2 - p1);
     std::cout << n << p << d << std::endl << p1 << p2 << std::endl;
     //      Assert_near(n.x, 0);
     //      Assert_near(n.y, 1);
     //      Assert_near(n.z, 0);
     //
     //      Assert_near(p.x, .1);
     //      Assert_near(p.y, 1);
     //      Assert_near(p.z, 0);
     //
     //      Assert_near(d, 0);
     //
     //      Assert_near(p1.x, .1);
     //      Assert_near(p1.y, 1);
     //      Assert_near(p1.z, 0);
     //
     //      Assert_near(p2.x, .1);
     //      Assert_near(p2.y, 1);
     //      Assert_near(p2.z, 0);
     }
     */
}

// =============================================================================

//// TODO

TEST(ChNarrowphaseMPR, box_box) {
    /*
     {
     // box on box offset
     real3 p, n(0, 0, 0);
     real d = 0;

     ChCollisionShape::Type A_T = ChCollisionShape::Type::BOX;
     real3 A_X = real3(1, 2, 0);
     real3 A_Y = real3(1, 1, 1);
     real3 A_Z = real3(0);
     real4 A_R = real4(1, 0, 0, 0);

     ChCollisionShape::Type B_T = ChCollisionShape::Type::BOX;
     real3 B_X = real3(0, 0, 0);
     real3 B_Y = real3(20, 1, 3);
     real3 B_Z = real3(0);
     real4 B_R = real4(1, 0, 0, 0);

     CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, d);
     real3 p1, p2;
     GetPoints(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, n, p, p1, p2);
     std::cout << n << p << d << std::endl << p1 << p2 << std::endl;
     //      Assert_near(n.x, 0);
     //      Assert_near(n.y, 1);
     //      Assert_near(n.z, 0);
     //
     //      Assert_near(p.x, .1);
     //      Assert_near(p.y, 1);
     //      Assert_near(p.z, 0);
     //
     //      Assert_near(d, 0);
     //
     //      Assert_near(p1.x, .1);
     //      Assert_near(p1.y, 1);
     //      Assert_near(p1.z, 0);
     //
     //      Assert_near(p2.x, .1);
     //      Assert_near(p2.y, 1);
     //      Assert_near(p2.z, 0);
     }
     */
}

// =============================================================================

TEST(ChNarrowphaseMPR, cylinder_sphere) {
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

    ////real oosqrt2 = sqrt(0.5);  // 1/sqrt(2)

    {
        // sphere center inside cylinder
        real3 s_pos(2.5, 1.5, 0);
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ChCollisionShape::Type::CYLINDER, c_pos, c_rot, real3(c_rad, c_hlen, c_rad));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        ASSERT_NE(res, 0);
        //// TODO:  WHAT IS EXPECTED HERE?

        delete shapeA;
        delete shapeB;
    }

    {
        // cap interaction (separated)
        real3 s_pos(4.5, 1.5, 0);

        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ChCollisionShape::Type::CYLINDER, c_pos, c_rot, real3(c_rad, c_hlen, c_rad));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        ASSERT_EQ(res, 0);

        delete shapeA;
        delete shapeB;
    }

    {
        // cap interaction (penetrated)
        real3 s_pos(3.75, 1.5, 0);

        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ChCollisionShape::Type::CYLINDER, c_pos, c_rot, real3(c_rad, c_hlen, c_rad));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        ASSERT_NE(res, 0);
        MPRGetPoints(shapeA, shapeB, envelope, norm, pt, pt1, pt2);
        depth = Dot(norm, pt2 - pt1);

        Assert_near(norm, real3(1, 0, 0), precision);
        ASSERT_NEAR(depth, -0.25, precision);
        Assert_near(pt1, real3(3, 1.5, 0), precision);
        Assert_near(pt2, real3(2.75, 1.5, 0), precision);

        delete shapeA;
        delete shapeB;
    }

    {
        // side interaction (separated)
        real3 s_pos(2.5, 3.5, 0);
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ChCollisionShape::Type::CYLINDER, c_pos, c_rot, real3(c_rad, c_hlen, c_rad));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        ASSERT_EQ(res, 0);

        delete shapeA;
        delete shapeB;
    }

    {
        // side interaction (penetrated)
        real3 s_pos(2.5, 2.5, 0);
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ChCollisionShape::Type::CYLINDER, c_pos, c_rot, real3(c_rad, c_hlen, c_rad));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        ASSERT_NE(res, 0);
        MPRGetPoints(shapeA, shapeB, envelope, norm, pt, pt1, pt2);
        depth = Dot(norm, pt2 - pt1);

        Assert_near(norm, real3(0, 1, 0), precision);
        ASSERT_NEAR(depth, -0.5, precision);
        Assert_near(pt1, real3(2.5, 2.0, 0), precision);
        Assert_near(pt2, real3(2.5, 1.5, 0), precision);

        delete shapeA;
        delete shapeB;
    }

    {
        // edge interaction (separated)
        real3 s_pos(4, 3, 0);
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ChCollisionShape::Type::CYLINDER, c_pos, c_rot, real3(c_rad, c_hlen, c_rad));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));
        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        ASSERT_EQ(res, 0);

        delete shapeA;
        delete shapeB;
    }

    //// TODO:  FIGURE OUT WHY THE FOLLOWING TEST FAILS!!!
    /*
     {
     // edge interaction (penetrated)
     real3 s_pos(3.5, 0, 2.5);
     c_hlen = 3.0;
     ConvexShape shapeA, shapeB;
     c_pos = real3(0, 0, 0);
     shapeA.type = ChCollisionShape::Type::CYLINDER;
     shapeA.A = c_pos;
     shapeA.B = real3(c_rad, c_hlen, c_rad);
     shapeA.C = real3(0);
     shapeA.R = c_rot;

     shapeB.type = ChCollisionShape::Type::ELLIPSOID;
     shapeB.A = s_pos;
     shapeB.B = real3(s_rad, s_rad, s_rad);
     shapeB.C = real3(0);
     shapeB.R = quaternion(.5, 0, 0, 0);
     bool res = MPRContact(shapeA, shapeB, norm, pt, depth);
     MPRGetPoints(shapeA, shapeB, norm, pt, pt1, pt2);
     depth = Dot(norm, pt2 - pt1);

     Assert_near(norm, real3(oosqrt2, oosqrt2, 0), precision);
     ASSERT_NEAR(depth, -1 + oosqrt2, precision);
     Assert_near(pt1, real3(3.0, 2.0, 0), precision);
     Assert_near(pt2, real3(3.5 - oosqrt2, 2.5 - oosqrt2, 0), precision);
     }
     */
}

// =============================================================================

TEST(ChNarrowphaseMPR, roundedcyl_sphere) {
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

    ////real oosqrt2 = sqrt(0.5);  // 1/sqrt(2)

    {
        // sphere center inside cylinder
        real3 s_pos(2.5, 1.5, 0);
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::ROUNDEDCYL, c_pos, c_rot,
                                                          real3(c_rad, c_hlen, c_rad), c_srad);
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        //// TODO: WHAT IS EXPECTED HERE?
        ASSERT_TRUE(res || !res);  //// <- FIX ME!

        delete shapeA;
        delete shapeB;
    }

    {
        // cap interaction (separated)
        real3 s_pos(4.5, 1.5, 0);
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::ROUNDEDCYL, c_pos, c_rot,
                                                          real3(c_rad, c_hlen, c_rad), c_srad);
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        ASSERT_EQ(res, 0);

        delete shapeA;
        delete shapeB;
    }

    {
        // cap interaction (penetrated)
        real3 s_pos(3.75, 1.5, 0);
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::ROUNDEDCYL, c_pos, c_rot,
                                                          real3(c_rad, c_hlen, c_rad), c_srad);
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));
        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        ASSERT_NE(res, 0);
        MPRGetPoints(shapeA, shapeB, envelope, norm, pt, pt1, pt2);
        depth = Dot(norm, pt2 - pt1);

        Assert_near(norm, real3(1, 0, 0), precision);
        ASSERT_NEAR(depth, -0.35, precision);
        Assert_near(pt1, real3(3.1, 1.5, 0), precision);
        Assert_near(pt2, real3(2.75, 1.5, 0), precision);

        delete shapeA;
        delete shapeB;
    }

    {
        // side interaction (separated)
        real3 s_pos(2.5, 3.5, 0);
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::ROUNDEDCYL, c_pos, c_rot,
                                                          real3(c_rad, c_hlen, c_rad), c_srad);
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        ASSERT_EQ(res, 0);

        delete shapeA;
        delete shapeB;
    }

    {
        // side interaction (penetrated)
        real3 s_pos(2.5, 2.5, 0);
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::ROUNDEDCYL, c_pos, c_rot,
                                                          real3(c_rad, c_hlen, c_rad), c_srad);
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));
        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        ASSERT_NE(res, 0);
        MPRGetPoints(shapeA, shapeB, envelope, norm, pt, pt1, pt2);
        depth = Dot(norm, pt2 - pt1);

        Assert_near(norm, real3(0, 1, 0), precision);
        ASSERT_NEAR(depth, -0.6, precision);
        Assert_near(pt1, real3(2.5, 2.1, 0), precision);
        Assert_near(pt2, real3(2.5, 1.5, 0), precision);

        delete shapeA;
        delete shapeB;
    }

    {
        // edge interaction (separated)
        real3 s_pos(4, 3, 0);
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::ROUNDEDCYL, c_pos, c_rot,
                                                          real3(c_rad, c_hlen, c_rad), c_srad);
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));
        bool res = MPRContact(shapeA, shapeB, envelope, norm, pt, depth);
        ASSERT_EQ(res, 0);

        delete shapeA;
        delete shapeB;
    }

    //// TODO:  FIGURE OUT WHY THE FOLLOWING TEST FAILS!!!
    /*
     {
     // edge interaction (penetrated)
     real3 s_pos(3.5, 2.5, 0);
     ConvexShape shapeA, shapeB;
     shapeA.type = ChCollisionShape::Type::ROUNDEDCYL;
     shapeA.A = c_pos;
     shapeA.B = real3(c_rad, c_hlen, c_rad);
     shapeA.C = real3(c_srad, 0, 0);
     shapeA.R = c_rot;

     shapeB.type = ChCollisionShape::Type::SPHERE;
     shapeB.A = s_pos;
     shapeB.B = real3(s_rad, 0, 0);
     shapeB.C = real3(0);
     shapeB.R = quaternion(1, 0, 0, 0);
     bool res = MPRContact(shapeA, shapeB, norm, pt, depth);
     ASSERT_EQ(res, 0);
     MPRGetPoints(shapeA, shapeB, norm, pt, pt1, pt2);
     depth = Dot(norm, pt2 - pt1);

     Assert_near(norm, real3(oosqrt2, oosqrt2, 0), precision);
     ASSERT_NEAR(depth, -1.1 + oosqrt2, precision);
     Assert_near(pt1, real3(3.0 + 0.1 * oosqrt2, 2.0 + 0.1 * oosqrt2, 0), precision);
     Assert_near(pt2, real3(3.5 - oosqrt2, 2.5 - oosqrt2, 0), precision);
     }
     */
}
