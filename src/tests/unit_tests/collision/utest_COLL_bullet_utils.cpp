// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Unit test for Bullet collision utility functions
// =============================================================================

#include "chrono/collision/ChCollisionUtilsBullet.h"
#include "chrono/collision/bullet/LinearMath/btTransform.h"

#include "gtest/gtest.h"

using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::endl;

const btScalar precision = btScalar(1e-6);

void Assert_eq(const btVector3& a, const btVector3& b) {
    ASSERT_EQ(a.x(), b.x());
    ASSERT_EQ(a.y(), b.y());
    ASSERT_EQ(a.z(), b.z());
}

TEST(BulletCollision, SnapPointToBox) {
    btVector3 hdims(1.0, 2.0, 3.0);

    {
        // interior point
        btVector3 loc(0.5, -1.0, 1.5);
        int code = bt_utils::SnapPointToBox(hdims, loc);
        ASSERT_EQ(code, 0);
        Assert_eq(loc, btVector3(0.5, -1.0, 1.5));
    }

    {
        // face point
        btVector3 loc(0.5, -1.0, -3.5);
        int code = bt_utils::SnapPointToBox(hdims, loc);
        ASSERT_EQ(code, 4);
        Assert_eq(loc, btVector3(0.5, -1.0, -3.0));
    }

    {
        // edge point
        btVector3 loc(0.5, -2.5, -3.5);
        int code = bt_utils::SnapPointToBox(hdims, loc);
        ASSERT_EQ(code, 6);
        Assert_eq(loc, btVector3(0.5, -2.0, -3.0));
    }

    {
        // vertex point
        btVector3 loc(1.5, -2.5, -3.5);
        int code = bt_utils::SnapPointToBox(hdims, loc);
        ASSERT_EQ(code, 7);
        Assert_eq(loc, btVector3(1.0, -2.0, -3.0));
    }
}

TEST(BulletCollision, PointInsideBox) {
    btVector3 hdims(1.0, 2.0, 3.0);

    {
        // interior point
        btVector3 loc(0.5, -1.0, 1.5);
        bool code = bt_utils::PointInsideBox(hdims, loc);
        ASSERT_EQ(code, true);
    }

    {
        // point in face Voronoi region
        btVector3 loc(0.5, -1.0, -3.5);
        bool code = bt_utils::PointInsideBox(hdims, loc);
        ASSERT_EQ(code, false);
    }

    {
        // point in edge Voronoi region
        btVector3 loc(0.5, -2.5, -3.5);
        bool code = bt_utils::PointInsideBox(hdims, loc);
        ASSERT_EQ(code, false);
    }

    {
        // point in vertex Voronoi region
        btVector3 loc(1.5, -2.5, -3.5);
        bool code = bt_utils::PointInsideBox(hdims, loc);
        ASSERT_EQ(code, false);
    }
}

TEST(BulletCollision, FindClosestBoxFace) {
    btVector3 hdims(1.0, 2.0, 3.0);

    {
        // interior, close to +x face
        btVector3 loc(0.5, -1.0, 1.5);
        int code = bt_utils::FindClosestBoxFace(hdims, loc);
        ASSERT_EQ(code, +1);
    }

    {
        // interior, close to -y face
        btVector3 loc(0.5, -1.75, 1.5);
        int code = bt_utils::FindClosestBoxFace(hdims, loc);
        ASSERT_EQ(code, -2);
    }

    {
        // exterior, close to +x face
        btVector3 loc(1.25, -1.0, 1.5);
        int code = bt_utils::FindClosestBoxFace(hdims, loc);
        ASSERT_EQ(code, +1);
    }

    {
        // exterior, close to -z face
        btVector3 loc(0.5, -1.0, -3.25);
        int code = bt_utils::FindClosestBoxFace(hdims, loc);
        ASSERT_EQ(code, -3);
    }
}

TEST(BulletCollision, IntersectSegmentBox) {
    btVector3 hdims(1.0, 2.0, 3.0);
    btScalar tMin;
    btScalar tMax;
    bool code;

    // Segment parallel to Y direction
    {
        btVector3 a(0, 1, 0);
        btScalar hlen = 1;

        // no intersection
        code = bt_utils::IntersectSegmentBox(hdims, btVector3(0.25, 3.25, 1.0), a, hlen, precision, tMin, tMax);
        ASSERT_EQ(code, false);

        // no intersection
        code = bt_utils::IntersectSegmentBox(hdims, btVector3(1.25, 1.5, 1.0), a, hlen, precision, tMin, tMax);
        ASSERT_EQ(code, false);

        // intersection (lower segment end inside box)
        code = bt_utils::IntersectSegmentBox(hdims, btVector3(0.25, 1.5, 1.0), a, hlen, precision, tMin, tMax);
        ASSERT_EQ(code, true);
        ASSERT_FLOAT_EQ(tMin, -hlen);
        ASSERT_FLOAT_EQ(tMax, 0.5);

        // intersection (upper segment end inside box)
        code = bt_utils::IntersectSegmentBox(hdims, btVector3(0.25, -1.5, 1.0), a, hlen, precision, tMin, tMax);
        ASSERT_EQ(code, true);
        ASSERT_FLOAT_EQ(tMin, -0.5);
        ASSERT_FLOAT_EQ(tMax, +hlen);

        // intersection (both segment ends inside box)
        code = bt_utils::IntersectSegmentBox(hdims, btVector3(0.25, 0.25, 1.0), a, hlen, precision, tMin, tMax);
        ASSERT_EQ(code, true);
        ASSERT_FLOAT_EQ(tMin, -hlen);
        ASSERT_FLOAT_EQ(tMax, +hlen);
    }

    //// TODO: add more tests
}

TEST(BulletCollision, IntersectLinePlane) {
    // Plane definition
    btVector3 pP(1, 1, 1);
    btVector3 pN(1, 1, 1);
    pN.normalize();

    // Line parallel to plane
    btVector3 lP1(1, 2, 3);
    btVector3 lD1(-0.25, -0.75, 1);
    lD1.normalize();

    // Line intersecting plane
    btVector3 lP2 = lP1;
    btVector3 lD2(+0.5, +0.5, 1);
    lD2.normalize();

    btScalar t;

    ASSERT_EQ(bt_utils::IntersectLinePlane(lP1, lD1, pP, pN, precision, t), false);
    ASSERT_EQ(bt_utils::IntersectLinePlane(lP2, lD2, pP, pN, precision, t), true);

    // Transform all points and directions with same transform
    btQuaternion q(1, 2, 3, 4);
    q.normalize();
    btVector3 c(-1, -2, -3);
    btTransform X(q, c);

    pP = X(pP);
    pN = X.getBasis() * pN;

    lP1 = X(lP1);
    lD1 = X.getBasis() * lD1;

    lP2 = X(lP2);
    lD2 = X.getBasis() * lD2;

    ASSERT_EQ(bt_utils::IntersectLinePlane(lP1, lD1, pP, pN, precision, t), false);
    ASSERT_EQ(bt_utils::IntersectLinePlane(lP2, lD2, pP, pN, precision, t), true);
}

void CheckSegmentCylinder(const btTransform& X) {
    // Cylinder definition (aligned with Y axis and centered at origin of specified transform)
    btVector3 cC = X(btVector3(0, 0, 0));
    btVector3 cD = X.getBasis() * btVector3(0, 1, 0);
    btScalar cH = 1;
    btScalar cR = 0.5;

    ASSERT_NEAR(cD.length(), 1, precision);

    btScalar tMin;
    btScalar tMax;
    bool code;

    // Segment parallel to cylinder axis.
    {
        btVector3 sC;
        btVector3 sD = X.getBasis() * btVector3(0, 1, 0);
        btScalar sH = 0.5;

        ASSERT_NEAR(sD.length(), 1, precision);

        // no intersection (outside cylindrical surface)
        sC = X(btVector3(0.25, 1.0, 1.0));
        code = bt_utils::IntersectSegmentCylinder(sC, sD, sH, cC, cD, cH, cR, precision, tMin, tMax);
        ASSERT_EQ(code, false);

        // no intersection (above cylindrical end-cap)
        sC = X(btVector3(0.25, 1.75, 0.25));
        code = bt_utils::IntersectSegmentCylinder(sC, sD, sH, cC, cD, cH, cR, precision, tMin, tMax);
        ASSERT_EQ(code, false);

        // intersection (lower segment end inside cylinder)
        sC = X(btVector3(0.25, 1.25, 0.25));
        code = bt_utils::IntersectSegmentCylinder(sC, sD, sH, cC, cD, cH, cR, precision, tMin, tMax);
        ASSERT_EQ(code, true);
        ASSERT_FLOAT_EQ(tMin, -sH);
        ASSERT_FLOAT_EQ(tMax, -0.25);

        // intersection (upper segment end inside cylinder)
        sC = X(btVector3(0.25, -1.25, 0.25));
        code = bt_utils::IntersectSegmentCylinder(sC, sD, sH, cC, cD, cH, cR, precision, tMin, tMax);
        ASSERT_EQ(code, true);
        ASSERT_FLOAT_EQ(tMin, +0.25);
        ASSERT_FLOAT_EQ(tMax, +sH);

        // intersection (both segment ends inside cylinder)
        sC = X(btVector3(0.25, 0.25, 0.25));
        code = bt_utils::IntersectSegmentCylinder(sC, sD, sH, cC, cD, cH, cR, precision, tMin, tMax);
        ASSERT_EQ(code, true);
        ASSERT_FLOAT_EQ(tMin, -sH);
        ASSERT_FLOAT_EQ(tMax, +sH);

        // intersection (both segment ends outside cylinder)
        sC = X(btVector3(0.25, 0.25, 0.25));
        sH = 3;
        code = bt_utils::IntersectSegmentCylinder(sC, sD, sH, cC, cD, cH, cR, precision, tMin, tMax);
        ASSERT_EQ(code, true);
        ASSERT_FLOAT_EQ(tMin, -1.25);
        ASSERT_FLOAT_EQ(tMax, +0.75);
    }

    // Segment parallel to cylinder caps.
    {
        btVector3 sC;
        btVector3 sD = X.getBasis() * btVector3(0.25, 0, -0.75).normalized();
        btScalar sH = 0.5;

        ASSERT_NEAR(sD.length(), 1, precision);
        ASSERT_NEAR(cD.dot(sD), 0, precision);

        // no intersection (outside cylindrical surface)
        sC = X(btVector3(1.0, 0.5, 2.0));
        code = bt_utils::IntersectSegmentCylinder(sC, sD, sH, cC, cD, cH, cR, precision, tMin, tMax);
        ASSERT_EQ(code, false);

        // no intersection (below cylindrical end-cap)
        sC = X(btVector3(0.25, -1.25, 0.25));
        code = bt_utils::IntersectSegmentCylinder(sC, sD, sH, cC, cD, cH, cR, precision, tMin, tMax);
        ASSERT_EQ(code, false);

        // intersection (positive segment end inside cylinder)
        sC = X(btVector3(-0.25, 0.25, 0.75));
        code = bt_utils::IntersectSegmentCylinder(sC, sD, sH, cC, cD, cH, cR, precision, tMin, tMax);
        ASSERT_EQ(code, true);
        ASSERT_FLOAT_EQ(tMax, +sH);
        ASSERT_GT(tMin, -sH);
        ASSERT_NEAR(bt_utils::DistancePointToLine(cC, cD, sC + tMin * sD), cR, precision);

        // Intersection (negative segment end inside cylinder)
        sC = X(btVector3(+0.25, 0.25, -0.75));
        code = bt_utils::IntersectSegmentCylinder(sC, sD, sH, cC, cD, cH, cR, precision, tMin, tMax);
        ASSERT_EQ(code, true);
        ASSERT_FLOAT_EQ(tMin, -sH);
        ASSERT_LT(tMax, +sH);
        ASSERT_NEAR(bt_utils::DistancePointToLine(cC, cD, sC + tMax * sD), cR, precision);

        // Intersection (both segment ends inside cylinder)
        sC = X(btVector3(0.25, 0.25, 0.25));
        sH = 0.125;
        code = bt_utils::IntersectSegmentCylinder(sC, sD, sH, cC, cD, cH, cR, precision, tMin, tMax);
        ASSERT_EQ(code, true);
        ASSERT_FLOAT_EQ(tMin, -sH);
        ASSERT_FLOAT_EQ(tMax, +sH);

        // Intersection (both segment ends outside cylinder)
        sC = X(btVector3(0.25, 0.25, 0.25));
        sH = 0.75;
        code = bt_utils::IntersectSegmentCylinder(sC, sD, sH, cC, cD, cH, cR, precision, tMin, tMax);
        ASSERT_EQ(code, true);
        ASSERT_GT(tMin, -sH);
        ASSERT_LT(tMax, +sH);
        ASSERT_NEAR(bt_utils::DistancePointToLine(cC, cD, sC + tMin * sD), cR, precision);
        ASSERT_NEAR(bt_utils::DistancePointToLine(cC, cD, sC + tMax * sD), cR, precision);
    }

    // General segment orientation
    {
        btVector3 sC;
        btVector3 sD = X.getBasis() * btVector3(0.25, 0.5, -0.75).normalized();
        btScalar sH = 0.5;

        ASSERT_NEAR(sD.length(), 1, precision);

        //// TODO: more tests here (no intersection)

        // intersection (both segment ends inside cylinder)
        sC = X(btVector3(0.25, 0.25, 0.25));
        sH = 0.125;
        code = bt_utils::IntersectSegmentCylinder(sC, sD, sH, cC, cD, cH, cR, precision, tMin, tMax);
        ASSERT_EQ(code, true);
        ASSERT_FLOAT_EQ(tMin, -sH);
        ASSERT_FLOAT_EQ(tMax, +sH);

        // Intersection (both segment ends outside cylinder)
        sC = X(btVector3(0.25, 0.25, 0.25));
        sH = 0.75;
        code = bt_utils::IntersectSegmentCylinder(sC, sD, sH, cC, cD, cH, cR, precision, tMin, tMax);
        ASSERT_EQ(code, true);
        ASSERT_GT(tMin, -sH);
        ASSERT_LT(tMax, +sH);
        ASSERT_NEAR(bt_utils::DistancePointToLine(cC, cD, sC + tMin * sD), cR, precision);
        ASSERT_NEAR(bt_utils::DistancePointToLine(cC, cD, sC + tMax * sD), cR, precision);
    }
}

TEST(BulletCollision, IntersectSegmentCylinder) {
    {
        btQuaternion q(0, 0, 0, 1);
        btVector3 c(0, 0, 0);
        btTransform X(q, c);
        CheckSegmentCylinder(X);
    }

    {
        btQuaternion q(1, 2, 3, 4);
        q.normalize();
        btVector3 c(-1, -2, -3);
        btTransform X(q, c);
        CheckSegmentCylinder(X);
    }
}
