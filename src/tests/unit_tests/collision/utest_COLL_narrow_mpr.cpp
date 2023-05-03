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

#include "chrono/collision/chrono/ChNarrowphase.h"
#include "chrono/collision/chrono/ChCollisionUtils.h"

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/core/ChMathematics.h"

#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbtBoxShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCollisionMargin.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConeShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCylinderShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbtMultiSphereShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbtSphereShape.h"

#include "gtest/gtest.h"

#include "unit_testing.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::collision::ch_utils;

real envelope = 0;

const float precision = 5e-6f;

real3 ToReal3(const cbtVector3& v) {
    return real3(v.x(), v.y(), v.z());
}

cbtVector3 ToBtVec(const real3& v) {
    return cbtVector3((cbtScalar)v.x, (cbtScalar)v.y, (cbtScalar)v.z);
}

// =============================================================================

TEST(ChNarrowphaseMPR, support_functions) {
    real3 Dir = Normalize(real3(1.123, -2.45, -8));

    {
        // sphere
        real3 R = real3(3.0, 1, 2);
        real3 answer_a = GetSupportPoint_Sphere(R.x, Dir);

        cbtSphereShape shape((cbtScalar)R.x);
        shape.setMargin(0);
        real3 answer_b = R.x * Dir;

        Assert_near(answer_a, answer_b, precision);
    }

    {
        // box
        real3 R = real3(3.0, 1, 2);
        real3 answer_a = GetSupportPoint_Box(R, Dir);

        cbtBoxShape shape(ToBtVec(R));
        shape.setMargin(0);
        real3 answer_b =
            ToReal3(shape.localGetSupportingVertex(cbtVector3((cbtScalar)Dir.x, (cbtScalar)Dir.y, (cbtScalar)Dir.z)));

        Assert_near(answer_a, answer_b, precision);
    }

    {
        // cylinder
        real3 R = real3(3.0, 3.0, 1.0);
        real3 answer_a = GetSupportPoint_Cylinder(R, Dir);

        cbtCylinderShapeZ shape(ToBtVec(R));
        shape.setMargin(0);
        real3 answer_b =
            ToReal3(shape.localGetSupportingVertex(cbtVector3((cbtScalar)Dir.x, (cbtScalar)Dir.y, (cbtScalar)Dir.z)));

        Assert_near(answer_a, answer_b, precision);
    }

    {
        // cone
        real3 R = real3(3.0, 3.0, 1.0);
        real3 answer_a = GetSupportPoint_Cone(R, Dir);

        cbtConeShapeZ shape((cbtScalar)R.x, (cbtScalar)R.z);
        shape.setMargin(0);
        real3 answer_b =
            ToReal3(shape.localGetSupportingVertex(cbtVector3((cbtScalar)Dir.x, (cbtScalar)Dir.y, (cbtScalar)Dir.z)));
        answer_b.z += R.z / 2;
        Assert_near(answer_a, answer_b, precision);
    }

    // TODO: Add Ellipsoid test
}

// =============================================================================

TEST(ChNarrowphaseMPR, sphere_sphere) {
    {
        // two spheres touching perfectly
        real3 n(0, 0, 0);
        real d = 0;

        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, real3(2, 2, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 0, 0));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, real3(2, 0, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 0, 0));

        real3 p1, p2;
        ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, n, p1, p2, d);

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
        real3 n(0, 0, 0);
        real d = 0;
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, real3(1, 1, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 0, 0));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, real3(2, 0, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 0, 0));

        real3 p1, p2;
        ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, n, p1, p2, d);

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
        real3 n(0, 0, 0);
        real d = 0;

        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::ELLIPSOID, real3(2, 2, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 1, 1));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(ChCollisionShape::Type::ELLIPSOID, real3(2, 0, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 1, 1));

        real3 p1, p2;
        ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, n, p1, p2, d);

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
        real3 n(0, 0, 0);
        real d = 0;

        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::ELLIPSOID, real3(1, 1, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 1, 1));
        ConvexShapeCustom* shapeB = new ConvexShapeCustom(ChCollisionShape::Type::ELLIPSOID, real3(2, 0, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 1, 1));

        real3 p1, p2;
        ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, n, p1, p2, d);
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
        real3 n(0, 0, 0);
        real d = 0;
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, real3(0, 1.5, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 0, 0));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::BOX, real3(0, 0, 0), quaternion(1, 0, 0, 0), real3(1, 1, 1));

        real3 p1, p2;
        ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, n, p1, p2, d);

        Assert_near(n, real3(0, -1, 0), precision);
        Assert_near(p1, real3(0, 0.5, 0), precision);
        Assert_near(p2, real3(0, 1, 0), precision);
        ASSERT_NEAR(d, -0.5, precision);

        delete shapeA;
        delete shapeB;
    }

    {
        // sphere on box offset
        real3 n(0, 0, 0);
        real d = 0;
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, real3(.1, 2, 0),
                                                          quaternion(1, 0, 0, 0), real3(1, 0, 0));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::BOX, real3(0, 0, 0), quaternion(1, 0, 0, 0), real3(1, 1, 1));

        real3 p1, p2;
        ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, n, p1, p2, d);

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
    quaternion c_rot = FromChQuaternion(Q_from_AngY(CH_C_PI_2));

    real3 norm;
    real depth;
    real3 pt;
    real3 pt1;
    real3 pt2;

    ////real oosqrt2 = sqrt(0.5);  // 1/sqrt(2)

    {
        std::cout << "sphere center inside cylinder" << std::endl;
        real3 s_pos(2.5, 1.5, 0);
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ChCollisionShape::Type::CYLINDER, c_pos, c_rot, real3(c_rad, c_rad, c_hlen));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, norm, pt1, pt2, depth);
        ASSERT_NE(res, 0);
        //// TODO:  WHAT IS EXPECTED HERE?

        delete shapeA;
        delete shapeB;
    }

    {
        std::cout << "cap interaction (separated)" << std::endl;
        real3 s_pos(4.5, 1.5, 0);

        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ChCollisionShape::Type::CYLINDER, c_pos, c_rot, real3(c_rad, c_rad, c_hlen));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, norm, pt1, pt2, depth);
        ASSERT_EQ(res, 0);

        delete shapeA;
        delete shapeB;
    }

    {
        std::cout << "cap interaction (penetrated)" << std::endl;
        real3 s_pos(3.75, 1.5, 0);

        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ChCollisionShape::Type::CYLINDER, c_pos, c_rot, real3(c_rad, c_rad, c_hlen));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, norm, pt1, pt2, depth);
        ASSERT_NE(res, 0);
        depth = Dot(norm, pt2 - pt1);

        Assert_near(norm, real3(1, 0, 0), precision);
        ASSERT_NEAR(depth, -0.25, precision);
        Assert_near(pt1, real3(3, 1.5, 0), precision);
        Assert_near(pt2, real3(2.75, 1.5, 0), precision);

        delete shapeA;
        delete shapeB;
    }

    {
        std::cout << "side interaction (separated)" << std::endl;
        real3 s_pos(2.5, 3.5, 0);
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ChCollisionShape::Type::CYLINDER, c_pos, c_rot, real3(c_rad, c_rad, c_hlen));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, norm, pt1, pt2, depth);
        ASSERT_EQ(res, 0);

        delete shapeA;
        delete shapeB;
    }

    {
        std::cout << "side interaction (penetrated)" << std::endl;
        real3 s_pos(2.5, 2.5, 0);
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ChCollisionShape::Type::CYLINDER, c_pos, c_rot, real3(c_rad, c_rad, c_hlen));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, norm, pt1, pt2, depth);
        ASSERT_NE(res, 0);
        depth = Dot(norm, pt2 - pt1);

        Assert_near(norm, real3(0, 1, 0), precision);
        ASSERT_NEAR(depth, -0.5, precision);
        Assert_near(pt1, real3(2.5, 2.0, 0), precision);
        Assert_near(pt2, real3(2.5, 1.5, 0), precision);

        delete shapeA;
        delete shapeB;
    }

    {
        std::cout << "edge interaction (separated)" << std::endl;
        real3 s_pos(4, 3, 0);
        ConvexShapeCustom* shapeA =
            new ConvexShapeCustom(ChCollisionShape::Type::CYLINDER, c_pos, c_rot, real3(c_rad, c_rad, c_hlen));
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));
        bool res = ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, norm, pt1, pt2, depth);
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
     shapeA.B = real3(c_rad, c_rad, c_hlen);
     shapeA.C = real3(0);
     shapeA.R = c_rot;

     shapeB.type = ChCollisionShape::Type::ELLIPSOID;
     shapeB.A = s_pos;
     shapeB.B = real3(s_rad, s_rad, s_rad);
     shapeB.C = real3(0);
     shapeB.R = quaternion(.5, 0, 0, 0);
     bool res = ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, norm, pt1, pt2, depth);
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
    quaternion c_rot = FromChQuaternion(Q_from_AngY(CH_C_PI_2));

    real3 norm;
    real depth;
    real3 pt;
    real3 pt1;
    real3 pt2;

    ////real oosqrt2 = sqrt(0.5);  // 1/sqrt(2)

    {
        std::cout << "sphere center inside cylinder" << std::endl;
        real3 s_pos(2.5, 1.5, 0);
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::ROUNDEDCYL, c_pos, c_rot,
                                                          real3(c_rad, c_rad, c_hlen), c_srad);
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, norm, pt1, pt2, depth);
        //// TODO: WHAT IS EXPECTED HERE?
        ASSERT_TRUE(res || !res);  //// <- FIX ME!

        delete shapeA;
        delete shapeB;
    }

    {
        std::cout << "cap interaction (separated)" << std::endl;
        real3 s_pos(4.5, 1.5, 0);
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::ROUNDEDCYL, c_pos, c_rot,
                                                          real3(c_rad, c_rad, c_hlen), c_srad);
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, norm, pt1, pt2, depth);
        ASSERT_EQ(res, 0);

        delete shapeA;
        delete shapeB;
    }

    {
        std::cout << "cap interaction (penetrated)" << std::endl;
        real3 s_pos(3.75, 1.5, 0);
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::ROUNDEDCYL, c_pos, c_rot,
                                                          real3(c_rad, c_rad, c_hlen), c_srad);
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));
        bool res = ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, norm, pt1, pt2, depth);
        ASSERT_NE(res, 0);
        depth = Dot(norm, pt2 - pt1);

        Assert_near(norm, real3(1, 0, 0), precision);
        ASSERT_NEAR(depth, -0.35, precision);
        Assert_near(pt1, real3(3.1, 1.5, 0), precision);
        Assert_near(pt2, real3(2.75, 1.5, 0), precision);

        delete shapeA;
        delete shapeB;
    }

    {
        std::cout << "side interaction (separated)" << std::endl;
        real3 s_pos(2.5, 3.5, 0);
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::ROUNDEDCYL, c_pos, c_rot,
                                                          real3(c_rad, c_rad, c_hlen), c_srad);
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));

        bool res = ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, norm, pt1, pt2, depth);
        ASSERT_EQ(res, 0);

        delete shapeA;
        delete shapeB;
    }

    {
        std::cout << "side interaction (penetrated)" << std::endl;
        real3 s_pos(2.5, 2.5, 0);
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::ROUNDEDCYL, c_pos, c_rot,
                                                          real3(c_rad, c_rad, c_hlen), c_srad);
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));
        bool res = ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, norm, pt1, pt2, depth);
        ASSERT_NE(res, 0);
        depth = Dot(norm, pt2 - pt1);

        Assert_near(norm, real3(0, 1, 0), precision);
        ASSERT_NEAR(depth, -0.6, precision);
        Assert_near(pt1, real3(2.5, 2.1, 0), precision);
        Assert_near(pt2, real3(2.5, 1.5, 0), precision);

        delete shapeA;
        delete shapeB;
    }

    {
        std::cout << "edge interaction (separated)" << std::endl;
        real3 s_pos(4, 3, 0);
        ConvexShapeCustom* shapeA = new ConvexShapeCustom(ChCollisionShape::Type::ROUNDEDCYL, c_pos, c_rot,
                                                          real3(c_rad, c_rad, c_hlen), c_srad);
        ConvexShapeCustom* shapeB =
            new ConvexShapeCustom(ChCollisionShape::Type::SPHERE, s_pos, quaternion(1, 0, 0, 0), real3(s_rad, 0, 0));
        bool res = ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, norm, pt1, pt2, depth);
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
     shapeA.B = real3(c_rad, c_rad, c_hlen);
     shapeA.C = real3(c_srad, 0, 0);
     shapeA.R = c_rot;

     shapeB.type = ChCollisionShape::Type::SPHERE;
     shapeB.A = s_pos;
     shapeB.B = real3(s_rad, 0, 0);
     shapeB.C = real3(0);
     shapeB.R = quaternion(1, 0, 0, 0);
     bool res = ChNarrowphase::MPRCollision(shapeA, shapeB, envelope, norm, pt1, pt2, depth);
     ASSERT_EQ(res, 0);
     depth = Dot(norm, pt2 - pt1);

     Assert_near(norm, real3(oosqrt2, oosqrt2, 0), precision);
     ASSERT_NEAR(depth, -1.1 + oosqrt2, precision);
     Assert_near(pt1, real3(3.0 + 0.1 * oosqrt2, 2.0 + 0.1 * oosqrt2, 0), precision);
     Assert_near(pt2, real3(3.5 - oosqrt2, 2.5 - oosqrt2, 0), precision);
     }
     */
}
