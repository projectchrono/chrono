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
// Authors: Radu Serban
// =============================================================================
//
// Chrono unit test for narrow phase type PRIMS collision detection
// =============================================================================

#include "chrono/collision/chrono/ChNarrowphase.h"
#include "chrono/collision/chrono/ChCollisionUtils.h"

#include "gtest/gtest.h"

#include "unit_testing.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::collision::ch_utils;

using std::cout;
using std::endl;

#ifdef CHRONO_MULTICORE_USE_DOUBLE
const double precision = 1e-10;
#else
const float precision = 1e-6f;
#endif

// =============================================================================
// Tests for various utility functions
// =============================================================================

TEST(ChNarrowphasePRIMS, snap_to_box) {
    real3 hdims(1.0, 2.0, 3.0);

    {
        // interior point
        real3 loc(0.5, -1.0, 1.5);
        int code = snap_to_box(hdims, loc);
        ASSERT_EQ(code, 0);
        Assert_eq(loc, real3(0.5, -1.0, 1.5));
    }

    {
        // face point
        real3 loc(0.5, -1.0, -3.5);
        int code = snap_to_box(hdims, loc);
        ASSERT_EQ(code, 4);
        Assert_eq(loc, real3(0.5, -1.0, -3.0));
    }

    {
        // edge point
        real3 loc(0.5, -2.5, -3.5);
        int code = snap_to_box(hdims, loc);
        ASSERT_EQ(code, 6);
        Assert_eq(loc, real3(0.5, -2.0, -3.0));
    }

    {
        // corner point
        real3 loc(1.5, -2.5, -3.5);
        int code = snap_to_box(hdims, loc);
        ASSERT_EQ(code, 7);
        Assert_eq(loc, real3(1.0, -2.0, -3.0));
    }
}

TEST(ChNarrowphasePRIMS, snap_to_cylinder) {
    real rad = 2;
    real hlen = 1.5;

    {
        // interior point
        real3 loc(0.5, 1.5, -1.0);
        int code = snap_to_cylinder(rad, hlen, loc);
        ASSERT_EQ(code, 0);
        Assert_eq(loc, real3(0.5, 1.5, -1.0));
    }

    {
        // cap point
        real3 loc(0.5, 1.5, 2.0);
        int code = snap_to_cylinder(rad, hlen, loc);
        ASSERT_EQ(code, 1);
        Assert_eq(loc, real3(0.5, 1.5, 1.5));
    }

    {
        // side point
        real3 loc(2.0, 1.0, 0.5);
        int code = snap_to_cylinder(rad, hlen, loc);
        ASSERT_EQ(code, 2);
        Assert_near(loc, real3(4 / sqrt(5.0), 2 / sqrt(5.0), 0.5), precision);
    }

    {
        // edge point
        real3 loc(2.0, 1.0, 2.0);
        int code = snap_to_cylinder(rad, hlen, loc);
        ASSERT_EQ(code, 3);
        Assert_near(loc, real3(4 / sqrt(5.0), 2 / sqrt(5.0), 1.5), precision);
    }
}

TEST(ChNarrowphasePRIMS, snap_to_box_face) {
    {
        {
            // z dir / in range
            real3 pt_on_box(1.0, 2.0, 3.0);
            real3 hdims(1.0, 2.0, 3.0);
            real3 pt_to_snap(0.1, 0.3, 2.0);
            uint code = 1 << 2;
            real3 result = snap_to_box_face(hdims, pt_on_box, code, pt_to_snap);
            Assert_near(result, real3(0.1, 0.3, 3.0), precision);
        }

        {
            // z dir / out of range
            real3 pt_on_box(1.0, 2.0, 3.0);
            real3 hdims(1.0, 2.0, 3.0);
            real3 pt_to_snap(1.7, -2.5, 4.0);
            uint code = 1 << 2;
            real3 result = snap_to_box_face(hdims, pt_on_box, code, pt_to_snap);
            Assert_near(result, real3(1.0, -2.0, 3.0), precision);
        }
    }
}

TEST(ChNarrowphasePRIMS, get_face_corners) {
    {
        {
            // dir x / inside
            real3 pt_on_box(1.0, 2.0, 3.0);
            real3 hdims(1.0, 2.0, 3.0);
            uint code = 1 << 0;
            real3 corners[4];

            get_face_corners(pt_on_box, code, corners);
            Assert_near(corners[0], real3(1.0, 2.0, 3.0), precision);
            Assert_near(corners[1], real3(1.0, -2.0, 3.0), precision);
            Assert_near(corners[2], real3(1.0, -2.0, -3.0), precision);
            Assert_near(corners[3], real3(1.0, 2.0, -3.0), precision);
        }

        {
            // dir y / inside
            real3 pt_on_box(1.0, 2.0, 3.0);
            real3 hdims(1.0, 2.0, 3.0);
            uint code = 1 << 1;
            real3 corners[4];

            get_face_corners(pt_on_box, code, corners);
            Assert_near(corners[0], real3(1.0, 2.0, 3.0), precision);
            Assert_near(corners[1], real3(1.0, 2.0, -3.0), precision);
            Assert_near(corners[2], real3(-1.0, 2.0, -3.0), precision);
            Assert_near(corners[3], real3(-1.0, 2.0, 3.0), precision);
        }

        {
            // dir z / inside
            real3 pt_on_box(1.0, 2.0, 3.0);
            real3 hdims(1.0, 2.0, 3.0);
            uint code = 1 << 2;
            real3 corners[4];

            get_face_corners(pt_on_box, code, corners);
            Assert_near(corners[0], real3(1, 2, 3), precision);
            Assert_near(corners[1], real3(-1, 2, 3), precision);
            Assert_near(corners[2], real3(-1, -2, 3), precision);
            Assert_near(corners[3], real3(1, -2, 3), precision);
        }
    }
}

TEST(ChNarrowphasePRIMS, get_edge_corners) {
    {
        {
            // dir x
            real3 pt_on_box(1, 2, 3);
            real3 hdims(1, 2, 3);
            uint code = 6;
            real3 corners[4];

            get_edge_corners(pt_on_box, code, corners);
            Assert_near(corners[0], real3(1, 2, 3), precision);
            Assert_near(corners[1], real3(-1, 2, 3), precision);
        }

        {
            // dir y
            real3 pt_on_box(1, -2, -3);
            real3 hdims(1, 2, 3);
            uint code = 5;
            real3 corners[4];

            get_edge_corners(pt_on_box, code, corners);
            Assert_near(corners[0], real3(1, -2, -3), precision);
            Assert_near(corners[1], real3(1, 2, -3), precision);
        }

        {
            // dir z
            real3 pt_on_box(-1, 2, -3);
            real3 hdims(1, 2, 3);
            uint code = 3;
            real3 corners[4];

            get_edge_corners(pt_on_box, code, corners);
            Assert_near(corners[0], real3(-1, 2, -3), precision);
            Assert_near(corners[1], real3(-1, 2, 3), precision);
        }
    }
}

TEST(ChNarrowphasePRIMS, point_vs_face) {
    real3 result;
    real3 normal;
    real dist;

    {
        // dir x | valid contact | separation = 0
        real3 pt_on_face(-0.9, 2, 3);
        real3 hdims(1, 2, 3);
        real3 pt_to_snap(-0.9, 1.4, 2);

        ASSERT_TRUE(point_vs_face(hdims, pt_on_face, 1, pt_to_snap, 0.0, result, normal, dist));
        Assert_near(result, real3(-1, 1.4, 2), precision);
        Assert_near(normal, real3(-1, 0, 0), precision);
        ASSERT_NEAR(dist, -0.1, precision);
    }

    {
        // dir y | valid contact
        real3 pt_on_face(1, 2, 3);
        real3 hdims(1, 2, 3);
        real3 pt_to_snap(0.5, 1.4, 2);

        ASSERT_TRUE(point_vs_face(hdims, pt_on_face, 2, pt_to_snap, 0.0, result, normal, dist));
        Assert_near(result, real3(0.5, 2, 2), precision);
        Assert_near(normal, real3(0, 1, 0), precision);
        ASSERT_NEAR(dist, -0.6, precision);
    }

    {
        // dir z | valid contact
        real3 pt_on_face(1, 2, 3);
        real3 hdims(1, 2, 3);
        real3 pt_to_snap(0.4, 0.8, 2.5);

        ASSERT_TRUE(point_vs_face(hdims, pt_on_face, 4, pt_to_snap, 0.0, result, normal, dist));
        Assert_near(result, real3(0.4, 0.8, 3), precision);
        Assert_near(normal, real3(0, 0, 1), precision);
        ASSERT_NEAR(dist, -0.5, precision);
    }

    {
        // dir x | invalid contact
        real3 pt_on_face(1, 2, 3);
        real3 hdims(1, 2, 3);
        real3 pt_to_snap(1.1, 1.4, 2);

        ASSERT_FALSE(point_vs_face(hdims, pt_on_face, 1, pt_to_snap, 0.0, result, normal, dist));
    }

    {
        // dir y | invalid contact
        real3 pt_on_face(1, 2, 3);
        real3 hdims(1, 2, 3);
        real3 pt_to_snap(0.5, 2.4, 2);

        ASSERT_FALSE(point_vs_face(hdims, pt_on_face, 2, pt_to_snap, 0.0, result, normal, dist));
    }

    {
        // dir z | invalid contact
        real3 pt_on_face(1, 2, 3);
        real3 hdims(1, 2, 3);
        real3 pt_to_snap(1.4, 0.8, 2.5);

        ASSERT_FALSE(point_vs_face(hdims, pt_on_face, 4, pt_to_snap, 0.0, result, normal, dist));
    }
}

TEST(ChNarrowphasePRIMS, segment_vs_edge) {
    real3 loc1;
    real3 loc2;
    {
        real3 pt_on_edge(1, 2, 3);
        real3 hdims(1, 2, 3);
        real3 pt_to_snap(0.4, 0.8, 2.5);
        real3 pt_1(0.5, 0.2, 2.5);
        real3 pt_2(1.5, 0.2, 2.5);

        ASSERT_TRUE(segment_vs_edge(hdims, pt_on_edge, 5, pt_1, pt_2, loc1, loc2));
        Assert_near(loc1, real3(1, 0.2, 3), precision);
        Assert_near(loc2, real3(1, 0.2, 2.5), precision);
    }
}

// =============================================================================
// Utility wrappers
// =============================================================================

void CheckValueList(real a[], const std::vector<real>& ref) {
    Assert_near(std::vector<real>(a, a + ref.size()), ref, precision);
}

void CheckValueList(real a[], int n, const real& ref) {
    CheckValueList(a, std::vector<real>(n, ref));
}

void CheckPointList(real3 a[], const std::vector<real3>& ref) {
    Assert_near(std::vector<real3>(a, a + ref.size()), ref, precision);
}

void CheckPointList(real3 a[], int n, const real3& ref) {
    CheckPointList(a, std::vector<real3>(n, ref));
}

// =============================================================================
// Tests for various primitive collision functions
// =============================================================================

class Collision : public ::testing::Test, public ::testing::WithParamInterface<bool> {
  public:
    Collision() : sep(GetParam()) {}

  protected:
    bool sep;
};

TEST_P(Collision, box_box) {
    real separation = sep ? 0.1 : 0.0;
    real penetration;
    real3 norm[8];
    real3 pt1[8];
    real3 pt2[8];
    real depth[8];
    real eff_rad[8];
    int nC = 0;

    // face to face | stack
    {
        real3 hdims1(1.0, 2.0, 3.0);
        real3 pos1(0.0, 0.0, 0.0);
        quaternion rot1 = quaternion(1, 0, 0, 0);

        real3 hdims2(1.0, 2.0, 3.0);
        real3 pos2(0.0, 0.0, 6.0);
        quaternion rot2 = quaternion(1, 0, 0, 0);

        ConvexShapeCustom* shape1 = new ConvexShapeCustom();
        shape1->type = ChCollisionShape::Type::BOX;
        shape1->position = pos1;
        shape1->dimensions = hdims1;
        shape1->rotation = rot1;

        ConvexShapeCustom* shape2 = new ConvexShapeCustom();
        shape2->type = ChCollisionShape::Type::BOX;
        shape2->position = pos2;
        shape2->dimensions = hdims2;
        shape2->rotation = rot2;

        // penetrated
        penetration = -0.05;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 8);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, 1));

        // penetrated, small penetration
        penetration = -1e-5;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 8);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, 1));

        // separated by less than 'separation'
        penetration = +0.05;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        if (sep) {
            ASSERT_EQ(nC, 8);
            CheckValueList(depth, nC, penetration);
            CheckPointList(norm, nC, real3(0, 0, 1));
        } else {
            ASSERT_EQ(nC, 0);
        }

        // separated by more than 'separation'
        penetration = +0.15;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // face to face | stack with rotation
    {
        real3 hdims1(1.0, 2.0, 3.0);
        real3 pos1(0.0, 0.0, 0.0);
        quaternion rot1 = quaternion(1, 0, 0, 0);

        real3 hdims2(1.0, 2.0, 3.0);
        real3 pos2(0.0, 0.0, 6.0);
        quaternion rot2 = FromChQuaternion(Q_from_AngZ(CH_C_PI_4));

        ConvexShapeCustom* shape1 = new ConvexShapeCustom();
        shape1->type = ChCollisionShape::Type::BOX;
        shape1->position = pos1;
        shape1->dimensions = hdims1;
        shape1->rotation = rot1;

        ConvexShapeCustom* shape2 = new ConvexShapeCustom();
        shape2->type = ChCollisionShape::Type::BOX;
        shape2->position = pos2;
        shape2->dimensions = hdims2;
        shape2->rotation = rot2;

        // penetrated
        penetration = -0.05;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 8);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, 1));

        // penetrated, small penetration
        penetration = -1e-5;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 8);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, 1));

        // separated by less than 'separation'
        penetration = +0.05;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        if (sep) {
            ASSERT_EQ(nC, 8);
            CheckValueList(depth, nC, penetration);
            CheckPointList(norm, nC, real3(0, 0, 1));
        } else {
            ASSERT_EQ(nC, 0);
        }

        // separated by more than 'separation'
        penetration = +0.15;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // face to face | stack with rotation and offset
    // 2 corners of top box on edges of bottom box
    {
        real3 hdims1(2.0, 2.0, 3.0);
        real3 pos1(0.0, 0.0, 0.0);
        quaternion rot1 = quaternion(1, 0, 0, 0);

        real3 hdims2(2.0, 2.0, 3.0);
        real3 pos2(2.0, 2.0, 6.0);
        quaternion rot2 = FromChQuaternion(Q_from_AngZ(CH_C_PI_4));

        ConvexShapeCustom* shape1 = new ConvexShapeCustom();
        shape1->type = ChCollisionShape::Type::BOX;
        shape1->position = pos1;
        shape1->dimensions = hdims1;
        shape1->rotation = rot1;

        ConvexShapeCustom* shape2 = new ConvexShapeCustom();
        shape2->type = ChCollisionShape::Type::BOX;
        shape2->position = pos2;
        shape2->dimensions = hdims2;
        shape2->rotation = rot2;

        // penetrated
        penetration = -0.05;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 3);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, 1));
        CheckPointList(pt1, {real3(2, 2 - 2 * sqrt(2.0), 3), real3(2 - 2 * sqrt(2.0), 2, 3), real3(2, 2, 3)});
        CheckPointList(pt2, {real3(2, 2 - 2 * sqrt(2.0), 3 + penetration), real3(2 - 2 * sqrt(2.0), 2, 3 + penetration),
                             real3(2, 2, 3 + penetration)});

        // penetrated, small penetration
        penetration = -1e-5;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 3);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, 1));
        CheckPointList(pt1, {real3(2, 2 - 2 * sqrt(2.0), 3), real3(2 - 2 * sqrt(2.0), 2, 3), real3(2, 2, 3)});
        CheckPointList(pt2, {real3(2, 2 - 2 * sqrt(2.0), 3 + penetration), real3(2 - 2 * sqrt(2.0), 2, 3 + penetration),
                             real3(2, 2, 3 + penetration)});

        // separated by less than 'separation'
        penetration = +0.05;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        if (sep) {
            ASSERT_EQ(nC, 3);
            CheckValueList(depth, nC, penetration);
            CheckPointList(norm, nC, real3(0, 0, 1));
            CheckPointList(pt1, {real3(2, 2 - 2 * sqrt(2.0), 3), real3(2 - 2 * sqrt(2.0), 2, 3), real3(2, 2, 3)});
            CheckPointList(pt2, {real3(2, 2 - 2 * sqrt(2.0), 3 + penetration),
                                 real3(2 - 2 * sqrt(2.0), 2, 3 + penetration), real3(2, 2, 3 + penetration)});
        } else {
            ASSERT_EQ(nC, 0);
        }

        // separated by more than 'separation'
        penetration = +0.15;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // face to face | stack with rotation and offset
    // edge-edge interactions
    {
        real3 hdims1(2.0, 2.0, 3.0);
        real3 pos1(0.0, 0.0, 0.0);
        quaternion rot1 = quaternion(1, 0, 0, 0);

        real3 hdims2(2.0, 2.0, 3.0);
        real3 pos2(2 * sqrt(2.0), 2 * sqrt(2.0), 6.0);
        quaternion rot2 = FromChQuaternion(Q_from_AngZ(CH_C_PI_4));

        ConvexShapeCustom* shape1 = new ConvexShapeCustom();
        shape1->type = ChCollisionShape::Type::BOX;
        shape1->position = pos1;
        shape1->dimensions = hdims1;
        shape1->rotation = rot1;

        ConvexShapeCustom* shape2 = new ConvexShapeCustom();
        shape2->type = ChCollisionShape::Type::BOX;
        shape2->position = pos2;
        shape2->dimensions = hdims2;
        shape2->rotation = rot2;

        // penetrated
        penetration = -0.05;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 3);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, 1));
        CheckPointList(pt1, {real3(2, 2 * sqrt(2.0) - 2, 3), real3(2 * sqrt(2.0) - 2, 2, 3), real3(2, 2, 3)});
        CheckPointList(pt2, {real3(2, 2 * sqrt(2.0) - 2, 3 + penetration), real3(2 * sqrt(2.0) - 2, 2, 3 + penetration),
                             real3(2, 2, 3 + penetration)});

        // penetrated, small penetration
        penetration = -1e-5;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 3);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, 1));
        CheckPointList(pt1, {real3(2, 2 * sqrt(2.0) - 2, 3), real3(2 * sqrt(2.0) - 2, 2, 3), real3(2, 2, 3)});
        CheckPointList(pt2, {real3(2, 2 * sqrt(2.0) - 2, 3 + penetration), real3(2 * sqrt(2.0) - 2, 2, 3 + penetration),
                             real3(2, 2, 3 + penetration)});

        // separated by less than 'separation'
        penetration = +0.05;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        if (sep) {
            ASSERT_EQ(nC, 3);
            CheckValueList(depth, nC, penetration);
            CheckPointList(norm, nC, real3(0, 0, 1));
            CheckPointList(pt1, {real3(2, 2 * sqrt(2.0) - 2, 3), real3(2 * sqrt(2.0) - 2, 2, 3), real3(2, 2, 3)});
            CheckPointList(pt2, {real3(2, 2 * sqrt(2.0) - 2, 3 + penetration),
                                 real3(2 * sqrt(2.0) - 2, 2, 3 + penetration), real3(2, 2, 3 + penetration)});
        } else {
            ASSERT_EQ(nC, 0);
        }

        // separated by more than 'separation'
        penetration = +0.15;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // face to face | small on large, in range
    {
        real3 hdims1(1.0, 2.0, 3.0);
        real3 pos1(0.0, 0.0, 0.0);
        quaternion rot1 = quaternion(1, 0, 0, 0);

        real3 hdims2(0.5, 1.0, 1.0);
        real3 pos2(0.0, 0.0, 4.0);
        quaternion rot2 = quaternion(1, 0, 0, 0);

        ConvexShapeCustom* shape1 = new ConvexShapeCustom();
        shape1->type = ChCollisionShape::Type::BOX;
        shape1->position = pos1;
        shape1->dimensions = hdims1;
        shape1->rotation = rot1;

        ConvexShapeCustom* shape2 = new ConvexShapeCustom();
        shape2->type = ChCollisionShape::Type::BOX;
        shape2->position = pos2;
        shape2->dimensions = hdims2;
        shape2->rotation = rot2;

        // penetrated
        penetration = -0.05;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 4);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, 1));
        CheckPointList(pt1, {real3(-0.5, -1, 3), real3(-0.5, 1, 3), real3(0.5, -1, 3), real3(0.5, 1, 3)});
        CheckPointList(pt2, {real3(-0.5, -1, 3 + penetration), real3(-0.5, 1, 3 + penetration),
                             real3(0.5, -1, 3 + penetration), real3(0.5, 1, 3 + penetration)});

        // penetrated, small penetration
        penetration = -1e-5;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 4);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, 1));
        CheckPointList(pt1, {real3(-0.5, -1, 3), real3(-0.5, 1, 3), real3(0.5, -1, 3), real3(0.5, 1, 3)});
        CheckPointList(pt2, {real3(-0.5, -1, 3 + penetration), real3(-0.5, 1, 3 + penetration),
                             real3(0.5, -1, 3 + penetration), real3(0.5, 1, 3 + penetration)});

        // separated by less than 'separation'
        penetration = +0.05;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        if (sep) {
            ASSERT_EQ(nC, 4);
            CheckValueList(depth, nC, penetration);
            CheckPointList(norm, nC, real3(0, 0, 1));
            CheckPointList(pt1, {real3(-0.5, -1, 3), real3(-0.5, 1, 3), real3(0.5, -1, 3), real3(0.5, 1, 3)});
            CheckPointList(pt2, {real3(-0.5, -1, 3 + penetration), real3(-0.5, 1, 3 + penetration),
                                 real3(0.5, -1, 3 + penetration), real3(0.5, 1, 3 + penetration)});
        } else {
            ASSERT_EQ(nC, 0);
        }

        // separated by more than 'separation'
        penetration = +0.15;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // face to edge | edge ends interacting with face
    // one large box at the bottom and a smaller one on top (rotated pi/4 about x)
    {
        real3 hdims1(1.0, 1.0, 1.0);
        real3 pos1(0.0, 0.0, 2.0 + 1.0 * sqrt(2.0));
        quaternion rot1 = FromChQuaternion(Q_from_AngX(CH_C_PI / 4));

        real3 hdims2(2.0, 2.0, 2.0);
        real3 pos2(0.0, 0.0, 0.0);
        quaternion rot2 = quaternion(1, 0, 0, 0);

        ConvexShapeCustom* shape1 = new ConvexShapeCustom();
        shape1->type = ChCollisionShape::Type::BOX;
        shape1->position = pos1;
        shape1->dimensions = hdims1;
        shape1->rotation = rot1;

        ConvexShapeCustom* shape2 = new ConvexShapeCustom();
        shape2->type = ChCollisionShape::Type::BOX;
        shape2->position = pos2;
        shape2->dimensions = hdims2;
        shape2->rotation = rot2;

        // penetrated
        penetration = -0.05;
        shape1->position = pos1 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 2);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, -1));
        CheckPointList(pt1, {real3(-1, 0, 2 + penetration), real3(1, 0, 2 + penetration)});
        CheckPointList(pt2, {real3(-1, 0, 2), real3(1, 0, 2)});

        // penetrated, small penetration
        penetration = -1e-5;
        shape1->position = pos1 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 2);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, -1));
        CheckPointList(pt1, {real3(-1, 0, 2 + penetration), real3(1, 0, 2 + penetration)});
        CheckPointList(pt2, {real3(-1, 0, 2), real3(1, 0, 2)});

        // separated by less than 'separation'
        penetration = +0.05;
        shape1->position = pos1 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        if (sep) {
            ASSERT_EQ(nC, 2);
            CheckValueList(depth, nC, penetration);
            CheckPointList(norm, nC, real3(0, 0, -1));
            CheckPointList(pt1, {real3(-1, 0, 2 + penetration), real3(1, 0, 2 + penetration)});
            CheckPointList(pt2, {real3(-1, 0, 2), real3(1, 0, 2)});
        } else {
            ASSERT_EQ(nC, 0);
        }

        // separated by more than 'separation'
        penetration = +0.15;
        shape1->position = pos1 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // face to edge | edge middle interacting with face
    // one large box at the bottom (rotated pi/4 about z) and a smaller one on top (rotated pi/4 about x)
    // translate small box in x and y so that its bottom edge intersects two edges of bottom box
    {
        real3 hdims1(1.0, 1.0, 1.0);
        real3 pos1(sqrt(2.0), sqrt(2.0), 2.0 + 1.0 * sqrt(2.0));
        quaternion rot1 = FromChQuaternion(Q_from_AngX(CH_C_PI / 4));

        real3 hdims2(2.0, 2.0, 2.0);
        real3 pos2(0.0, 0.0, 0.0);
        quaternion rot2 = FromChQuaternion(Q_from_AngZ(CH_C_PI / 4));

        ConvexShapeCustom* shape1 = new ConvexShapeCustom();
        shape1->type = ChCollisionShape::Type::BOX;
        shape1->position = pos1;
        shape1->dimensions = hdims1;
        shape1->rotation = rot1;

        ConvexShapeCustom* shape2 = new ConvexShapeCustom();
        shape2->type = ChCollisionShape::Type::BOX;
        shape2->position = pos2;
        shape2->dimensions = hdims2;
        shape2->rotation = rot2;

        // penetrated
        penetration = -0.05;
        shape1->position = pos1 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 2);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, -1));
        CheckPointList(
            pt1, {real3(sqrt(2.0) - 1, sqrt(2.0), 2 + penetration), real3(sqrt(2.0), sqrt(2.0), 2 + penetration)});
        CheckPointList(pt2, {real3(sqrt(2.0) - 1, sqrt(2.0), 2), real3(sqrt(2.0), sqrt(2.0), 2)});

        // penetrated, small penetration
        penetration = -1e-5;
        shape1->position = pos1 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 2);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, -1));
        CheckPointList(
            pt1, {real3(sqrt(2.0) - 1, sqrt(2.0), 2 + penetration), real3(sqrt(2.0), sqrt(2.0), 2 + penetration)});
        CheckPointList(pt2, {real3(sqrt(2.0) - 1, sqrt(2.0), 2), real3(sqrt(2.0), sqrt(2.0), 2)});

        // separated by less than 'separation'
        penetration = +0.05;
        shape1->position = pos1 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        if (sep) {
            ASSERT_EQ(nC, 2);
            CheckValueList(depth, nC, penetration);
            CheckPointList(norm, nC, real3(0, 0, -1));
            CheckPointList(
                pt1, {real3(sqrt(2.0) - 1, sqrt(2.0), 2 + penetration), real3(sqrt(2.0), sqrt(2.0), 2 + penetration)});
            CheckPointList(pt2, {real3(sqrt(2.0) - 1, sqrt(2.0), 2), real3(sqrt(2.0), sqrt(2.0), 2)});
        } else {
            ASSERT_EQ(nC, 0);
        }

        // separated by more than 'separation'
        penetration = +0.15;
        shape1->position = pos1 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // Face-corner
    {
        real3 hdims1(1.0, 1.0, 1.0);
        real3 pos1(0.0, 0.0, 0.0);
        quaternion rot1(1, 0, 0, 0);

        real3 hdims2(1.0, 1.0, 1.0);
        real3 pos2(0.5, 0.5, 1.0 + sqrt(3.0));
        quaternion rot2 = FromChQuaternion(Q_from_AngAxis(atan(sqrt(2.0)), ChVector<>(1, 1, 0).GetNormalized()));

        ConvexShapeCustom* shape1 = new ConvexShapeCustom();
        shape1->type = ChCollisionShape::Type::BOX;
        shape1->position = pos1;
        shape1->dimensions = hdims1;
        shape1->rotation = rot1;

        ConvexShapeCustom* shape2 = new ConvexShapeCustom();
        shape2->type = ChCollisionShape::Type::BOX;
        shape2->position = pos2;
        shape2->dimensions = hdims2;
        shape2->rotation = rot2;

        // penetrated
        penetration = -0.05;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 1);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, 1));
        CheckPointList(pt1, {real3(0.5, 0.5, 1)});
        CheckPointList(pt2, {real3(0.5, 0.5, 1 + penetration)});

        // penetrated, small penetration
        penetration = -1e-5;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 1);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, 1));
        CheckPointList(pt1, {real3(0.5, 0.5, 1)});
        CheckPointList(pt2, {real3(0.5, 0.5, 1 + penetration)});

        // separated by less than 'separation'
        penetration = +0.05;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        if (sep) {
            ASSERT_EQ(nC, 1);
            CheckValueList(depth, nC, penetration);
            CheckPointList(norm, nC, real3(0, 0, 1));
            CheckPointList(pt1, {real3(0.5, 0.5, 1)});
            CheckPointList(pt2, {real3(0.5, 0.5, 1 + penetration)});
        } else {
            ASSERT_EQ(nC, 0);
        }

        // separated by more than 'separation'
        penetration = +0.15;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // Edge-edge
    // Bottom box rotated by 45 deg about y. Top box rotated by 45 deg about x.
    {
        real3 hdims1(1.0, 1.0, 1.0);
        real3 pos1(0, 0, 0);
        quaternion rot1 = FromChQuaternion(Q_from_AngY(CH_C_PI / 4));

        real3 hdims2(1.0, 1.0, 1.0);
        real3 pos2(0.0, 0.0, 2 * sqrt(2.0));
        quaternion rot2 = FromChQuaternion(Q_from_AngX(CH_C_PI / 4));

        ConvexShapeCustom* shape1 = new ConvexShapeCustom();
        shape1->type = ChCollisionShape::Type::BOX;
        shape1->position = pos1;
        shape1->dimensions = hdims1;
        shape1->rotation = rot1;

        ConvexShapeCustom* shape2 = new ConvexShapeCustom();
        shape2->type = ChCollisionShape::Type::BOX;
        shape2->position = pos2;
        shape2->dimensions = hdims2;
        shape2->rotation = rot2;

        // penetrated
        penetration = -0.05;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 1);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, 1));
        CheckPointList(pt1, {real3(0, 0, sqrt(2.0))});
        CheckPointList(pt2, {real3(0, 0, sqrt(2.0) + penetration)});

        // penetrated, small penetration
        penetration = -1e-5;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 1);
        CheckValueList(depth, nC, penetration);
        CheckPointList(norm, nC, real3(0, 0, 1));
        CheckPointList(pt1, {real3(0, 0, sqrt(2.0))});
        CheckPointList(pt2, {real3(0, 0, sqrt(2.0) + penetration)});

        // separated by less than 'separation'
        penetration = +0.05;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        if (sep) {
            ASSERT_EQ(nC, 1);
            CheckValueList(depth, nC, penetration);
            CheckPointList(norm, nC, real3(0, 0, 1));
            CheckPointList(pt1, {real3(0, 0, sqrt(2.0))});
            CheckPointList(pt2, {real3(0, 0, sqrt(2.0) + penetration)});
        } else {
            ASSERT_EQ(nC, 0);
        }

        // separated by more than 'separation'
        penetration = +0.15;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // Corner-corner
    // Notes:
    // - the current algorithm cannot produce a proper corner-corner interaction. Because it always picks
    //   the direction of minimum overlap, for this case it will report a face-face interaction.
    // - for similar reasons, the current algorithm will not report an interaction for a configuration
    //   where two boxes are separated, but with corners are in each others Voronoi region (no matter of
    //   the separation distance.
    {
        real3 hdims1(1.0, 1.0, 1.0);
        real3 pos1(0.0, 0.0, 0.0);
        quaternion rot1 = FromChQuaternion(Q_from_AngAxis(atan(sqrt(2.0)), ChVector<>(1, 1, 0).GetNormalized()));

        real3 hdims2(1.0, 1.0, 1.0);
        real3 pos2(0, 0, sqrt(3.0) + sqrt(3.0));
        quaternion rot2 = FromChQuaternion(Q_from_AngAxis(atan(sqrt(2.0)), ChVector<>(1, 1, 0).GetNormalized()));

        ConvexShapeCustom* shape1 = new ConvexShapeCustom();
        shape1->type = ChCollisionShape::Type::BOX;
        shape1->position = pos1;
        shape1->dimensions = hdims1;
        shape1->rotation = rot1;

        ConvexShapeCustom* shape2 = new ConvexShapeCustom();
        shape2->type = ChCollisionShape::Type::BOX;
        shape2->position = pos2;
        shape2->dimensions = hdims2;
        shape2->rotation = rot2;

        // penetrated
        penetration = -0.05;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 4);
        CheckValueList(depth, nC, penetration / std::sqrt(3.0));
        ////for (int i = 0; i < 4; i++) {
        ////    real3 p1 = RotateT((pt1[i] - pos1), rot1);
        ////    real3 p2 = RotateT((pt2[i] - pos2 - real3(0, 0, penetration)), rot2);
        ////}

        // penetrated, small penetration
        penetration = -1e-5 * std::sqrt(3.0);
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        ASSERT_EQ(nC, 4);
        CheckValueList(depth, nC, penetration / std::sqrt(3.0));

        // separated by less than 'separation'
        penetration = +0.05;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shape1, shape2, separation, norm, pt1, pt2, depth, eff_rad, nC));
        if (sep) {
            // Limitation of current algorithm
            ASSERT_EQ(nC, 0);
        } else {
            ASSERT_EQ(nC, 0);
        }

        // separated by more than 'separation'
        penetration = +0.15;
        shape2->position = pos2 + real3(0, 0, penetration);
        ASSERT_EQ(nC, 0);
    }
}

TEST_P(Collision, sphere_sphere) {
    ConvexShapeCustom* shapeS1 = new ConvexShapeCustom();
    shapeS1->type = ChCollisionShape::Type::SPHERE;
    shapeS1->radius = 0;
    shapeS1->rotation = quaternion(1, 0, 0, 0);

    ConvexShapeCustom* shapeS2 = new ConvexShapeCustom();
    shapeS2->type = ChCollisionShape::Type::SPHERE;
    shapeS2->radius = 0;
    shapeS2->rotation = quaternion(1, 0, 0, 0);

    real separation = sep ? 0.1 : 0.0;

    // Output quantities.
    real3 norm;
    real3 pt1;
    real3 pt2;
    real depth;
    real eff_rad;
    int nC;

    // separated (far)
    {
        shapeS1->position = real3(2, 2, 0);
        shapeS1->dimensions = real3(1, 0, 0);

        shapeS2->position = real3(2, 0, 0);
        shapeS2->dimensions = real3(0.5, 0, 0);

        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeS1, shapeS2, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // separated (near)
    {
        shapeS1->position = real3(2, 2, 0);
        shapeS1->dimensions = real3(1, 0, 0);

        shapeS2->position = real3(2, 0, 0);
        shapeS2->dimensions = real3(0.95, 0, 0);

        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeS1, shapeS2, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        if (sep) {
            ASSERT_EQ(nC, 1);
            Assert_near(norm, real3(0, -1, 0), precision);
            ASSERT_NEAR(depth, 0.05, precision);
            Assert_near(pt1, real3(2, 1, 0), precision);
            Assert_near(pt2, real3(2, 0.95, 0), precision);
            ASSERT_NEAR(eff_rad, 0.95 / 1.95, precision);
        } else {
            ASSERT_EQ(nC, 0);
        }
    }

    // touching
    {
        shapeS1->position = real3(2, 2, 0);
        shapeS1->dimensions = real3(1, 0, 0);

        shapeS2->position = real3(2, 0, 0);
        shapeS2->dimensions = real3(1, 0, 0);

        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeS1, shapeS2, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        if (sep) {
            ASSERT_EQ(nC, 1);
            Assert_near(norm, real3(0, -1, 0), precision);
            ASSERT_NEAR(depth, 0, precision);
            Assert_near(pt1, real3(2, 1, 0), precision);
            Assert_near(pt2, real3(2, 1, 0), precision);
            ASSERT_NEAR(eff_rad, 0.5, precision);
        } else {
            ASSERT_EQ(nC, 0);
        }
    }

    // penetrated
    {
        shapeS1->position = real3(1, 1, 0);
        shapeS1->dimensions = real3(1, 0, 0);

        shapeS2->position = real3(2.5, 1, 0);
        shapeS2->dimensions = real3(1, 0, 0);

        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeS1, shapeS2, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 1);
        Assert_near(norm, real3(1, 0, 0), precision);
        ASSERT_NEAR(depth, -0.5, precision);
        Assert_near(pt1, real3(2, 1, 0), precision);
        Assert_near(pt2, real3(1.5, 1, 0), precision);
        ASSERT_NEAR(eff_rad, 0.5, precision);
    }

    delete shapeS1;
    delete shapeS2;
}

// -----------------------------------------------------------------------------

TEST_P(Collision, box_sphere) {
    // Fictitious radius of curvature for corners and edges
    real edge_radius = ChNarrowphase::GetDefaultEdgeRadius();

    // Box position and orientation fixed for all tests.
    // Rotated by 45 degrees around Z axis and shifted by sqrt(2) in X direction.
    real3 b_hdims(1.0, 2.0, 3.0);
    real3 b_pos(sqrt(2.0), 0.0, 0.0);
    quaternion b_rot = FromChQuaternion(Q_from_AngZ(CH_C_PI_4));

    ConvexShapeCustom* shapeC = new ConvexShapeCustom();
    shapeC->type = ChCollisionShape::Type::BOX;
    shapeC->position = b_pos;
    shapeC->dimensions = b_hdims;
    shapeC->radius = 0;
    shapeC->rotation = b_rot;

    // Sphere position changes for each test.
    real s_rad = 1.5;  // sphere radius

    ConvexShapeCustom* shapeS = new ConvexShapeCustom();
    shapeS->type = ChCollisionShape::Type::SPHERE;
    shapeS->dimensions = real3(s_rad, 0, 0);
    shapeS->radius = 0;
    shapeS->rotation = quaternion(1, 0, 0, 0);

    real separation = sep ? 0.1 : 0.0;

    // Output quantities.
    real3 norm;
    real3 pt1;
    real3 pt2;
    real depth;
    real eff_rad;
    int nC;

    real oosqrt2 = sqrt(0.5);  // 1/sqrt(2)

    // sphere center inside box
    {
        shapeS->position = real3(0.5, 0.5, 1.0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // face interaction (separated far)
    {
        shapeS->position = real3(3.5, 2.5, 1.0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // face interaction (separated near)
    {
        shapeS->position = real3(4.55 * oosqrt2, 2.55 * oosqrt2, 1.0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        if (sep) {
            ASSERT_EQ(nC, 1);
            Assert_near(norm, real3(oosqrt2, oosqrt2, 0.0), precision);
            ASSERT_NEAR(depth, 0.05, precision);
            Assert_near(pt1, real3(3.0 * oosqrt2, oosqrt2, 1.0), precision);
            Assert_near(pt2, real3(3.05 * oosqrt2, 1.05 * oosqrt2, 1.0), precision);
        } else {
            ASSERT_EQ(nC, 0);
        }
    }

    // face interaction (penetrated)
    {
        shapeS->position = real3(4 * oosqrt2, 2.0 * oosqrt2, 1.0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 1);
        Assert_near(norm, real3(oosqrt2, oosqrt2, 0.0), precision);
        ASSERT_NEAR(depth, -0.5, precision);
        Assert_near(pt1, real3(3.0 * oosqrt2, oosqrt2, 1.0), precision);
        Assert_near(pt2, real3(2.5 * oosqrt2, 0.5 * oosqrt2, 1.0), precision);
        ASSERT_NEAR(eff_rad, s_rad, precision);
    }

    // edge interaction (separated far)
    {
        shapeS->position = real3(oosqrt2, 4.0, 1.0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // edge interaction (separated near)
    {
        shapeS->position = real3(oosqrt2, 3.0 * oosqrt2 + 1.55, 1.0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        if (sep) {
            ASSERT_EQ(nC, 1);
            Assert_near(norm, real3(0.0, 1.0, 0.0), precision);
            ASSERT_NEAR(depth, 0.05, precision);
            Assert_near(pt1, real3(oosqrt2, 3.0 * oosqrt2, 1.0), precision);
            Assert_near(pt2, real3(oosqrt2, 3.0 * oosqrt2 + 0.05, 1.0), precision);
        } else {
            ASSERT_EQ(nC, 0);
        }
    }

    // edge interaction (penetrated)
    {
        shapeS->position = real3(oosqrt2, 3.0 * oosqrt2 + 1.0, 1.0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 1);
        Assert_near(norm, real3(0.0, 1.0, 0.0), precision);
        ASSERT_NEAR(depth, -0.5, precision);
        Assert_near(pt1, real3(oosqrt2, 3.0 * oosqrt2, 1.0), precision);
        Assert_near(pt2, real3(oosqrt2, 3.0 * oosqrt2 - 0.5, 1.0), precision);
        ASSERT_NEAR(eff_rad, s_rad * edge_radius / (s_rad + edge_radius), precision);
    }

    // corner interaction (separated far)
    {
        shapeS->position = real3(oosqrt2, 4.0, 4.0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // corner interaction (separated near)
    {
        real3 s_pos(oosqrt2, 4.55 * oosqrt2, 3.0 + 1.55 * oosqrt2);
        shapeS->position = s_pos;
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        if (sep) {
            ASSERT_EQ(nC, 1);
            Assert_near(norm, real3(0.0, oosqrt2, oosqrt2), precision);
            ASSERT_NEAR(depth, 0.05, precision);
            Assert_near(pt1, real3(oosqrt2, 3.0 * oosqrt2, 3.0), precision);
            Assert_near(pt2, s_pos - s_rad * norm, precision);
        } else {
            ASSERT_EQ(nC, 0);
        }
    }

    // corner interaction (penetrated)
    {
        real3 s_pos(oosqrt2, 4.0 * oosqrt2, 3.0 + oosqrt2);
        shapeS->position = s_pos;
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 1);
        Assert_near(norm, real3(0.0, oosqrt2, oosqrt2), precision);
        ASSERT_NEAR(depth, -0.5, precision);
        Assert_near(pt1, real3(oosqrt2, 3.0 * oosqrt2, 3.0), precision);
        Assert_near(pt2, s_pos - s_rad * norm, precision);
        ASSERT_NEAR(eff_rad, s_rad * edge_radius / (s_rad + edge_radius), precision);
    }

    delete shapeS;
    delete shapeC;
}

// -----------------------------------------------------------------------------

//// TODO: include case with non-zero separation
TEST_P(Collision, capsule_sphere) {
    // Capsule position and orientation fixed for all tests.
    // aligned with X axis and shifted by its half-length in the X direction.
    real c_rad = 0.5;
    real c_hlen = 2.0;
    real3 c_pos(c_hlen, 0, 0);
    quaternion c_rot = FromChQuaternion(Q_from_AngY(CH_C_PI_2));

    ConvexShapeCustom* shapeC = new ConvexShapeCustom();
    shapeC->type = ChCollisionShape::Type::CAPSULE;
    shapeC->position = c_pos;
    shapeC->dimensions = real3(c_rad, c_hlen, c_rad);
    shapeC->radius = 0;
    shapeC->rotation = c_rot;

    // Sphere position changes for each test.
    real s_rad = 1.0;  // sphere radius

    ConvexShapeCustom* shapeS = new ConvexShapeCustom();
    shapeS->type = ChCollisionShape::Type::SPHERE;
    shapeS->dimensions = real3(s_rad, 0, 0);
    shapeS->radius = 0;
    shapeS->rotation = quaternion(1, 0, 0, 0);

    // Output quantities.
    real3 norm;
    real3 pt1;
    real3 pt2;
    real depth;
    real eff_rad;
    int nC;

    real oosqrt2 = sqrt(0.5);  // 1/sqrt(2)

    // sphere center on capsule axis
    {
        shapeS->position = real3(3.0, 0.0, 0.0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // cap interaction (separated)
    {
        shapeS->position = real3(5.0, 1.5, 0.0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // cap interaction (penetrated)"
    {
        shapeS->position = real3(5.0, 1.0, 0.0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 1);
        Assert_near(norm, real3(oosqrt2, oosqrt2, 0), precision);
        ASSERT_NEAR(depth, sqrt(2.0) - 1.5, precision);
        Assert_near(pt1, real3(4.0 + 0.5 * oosqrt2, 0.5 * oosqrt2, 0), precision);
        Assert_near(pt2, real3(5.0 - oosqrt2, 1.0 - oosqrt2, 0), precision);
        ASSERT_NEAR(eff_rad, s_rad * c_rad / (s_rad + c_rad), precision);
    }

    // side interaction (separated)
    {
        shapeS->position = real3(2.5, 2.0, 0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // side interaction (penetrated)
    {
        shapeS->position = real3(2.5, 1.25, 0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 1);
        Assert_near(norm, real3(0, 1, 0), precision);
        ASSERT_NEAR(depth, -0.25, precision);
        Assert_near(pt1, real3(2.5, 0.5, 0), precision);
        Assert_near(pt2, real3(2.5, 0.25, 0), precision);
        ASSERT_NEAR(eff_rad, s_rad * c_rad / (s_rad + c_rad), precision);
    }

    delete shapeS;
    delete shapeC;
}

// -----------------------------------------------------------------------------

//// TODO: include case with non-zero separation
TEST_P(Collision, cylinder_sphere) {
    // Fictitious radius of curvature for corners and edges
    real edge_radius = ChNarrowphase::GetDefaultEdgeRadius();

    // Cylinder position and orientation fixed for all tests.
    // Aligned with X axis and shifted by its half-length in the X direction.
    real c_rad = 2.0;
    real c_hlen = 1.5;
    real3 c_pos(c_hlen, 0, 0);
    quaternion c_rot = FromChQuaternion(Q_from_AngY(CH_C_PI_2));

    ConvexShapeCustom* shapeC = new ConvexShapeCustom();
    shapeC->type = ChCollisionShape::Type::CYLINDER;
    shapeC->position = c_pos;
    shapeC->dimensions = real3(c_rad, c_hlen, c_rad);
    shapeC->radius = 0;
    shapeC->rotation = c_rot;

    // Sphere position changes for each test.
    real s_rad = 1.0;  // sphere radius

    ConvexShapeCustom* shapeS = new ConvexShapeCustom();
    shapeS->type = ChCollisionShape::Type::SPHERE;
    shapeS->dimensions = real3(s_rad, 0, 0);
    shapeS->radius = 0;
    shapeS->rotation = quaternion(1, 0, 0, 0);

    // Output quantities.
    real3 norm;
    real3 pt1;
    real3 pt2;
    real depth;
    real eff_rad;
    int nC;

    real oosqrt2 = sqrt(0.5);  // 1/sqrt(2)

    // sphere center inside cylinder
    {
        shapeS->position = real3(2.5, 1.5, 0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // cap interaction (separated)
    {
        shapeS->position = real3(4.5, 1.5, 0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // cap interaction (penetrated)
    {
        shapeS->position = real3(3.75, 1.5, 0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 1);
        Assert_near(norm, real3(1, 0, 0), precision);
        ASSERT_NEAR(depth, -0.25, precision);
        Assert_near(pt1, real3(3, 1.5, 0), precision);
        Assert_near(pt2, real3(2.75, 1.5, 0), precision);
        ASSERT_NEAR(eff_rad, s_rad, precision);
    }

    // side interaction (separated)
    {
        shapeS->position = real3(2.5, 3.5, 0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // side interaction (penetrated)
    {
        shapeS->position = real3(2.5, 2.5, 0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 1);
        Assert_near(norm, real3(0, 1, 0), precision);
        ASSERT_NEAR(depth, -0.5, precision);
        Assert_near(pt1, real3(2.5, 2.0, 0), precision);
        Assert_near(pt2, real3(2.5, 1.5, 0), precision);
        ASSERT_NEAR(eff_rad, s_rad * c_rad / (s_rad + c_rad), precision);
    }

    // edge interaction (separated)
    {
        shapeS->position = real3(4, 3, 0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // edge interaction (penetrated)
    {
        shapeS->position = real3(3.5, 2.5, 0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 1);
        Assert_near(norm, real3(oosqrt2, oosqrt2, 0), precision);
        ASSERT_NEAR(depth, -1 + oosqrt2, precision);
        Assert_near(pt1, real3(3.0, 2.0, 0), precision);
        Assert_near(pt2, real3(3.5 - oosqrt2, 2.5 - oosqrt2, 0), precision);
        ASSERT_NEAR(eff_rad, s_rad * edge_radius / (s_rad + edge_radius), precision);
    }

    delete shapeS;
    delete shapeC;
}

// -----------------------------------------------------------------------------

//// TODO: include case with non-zero separation
TEST_P(Collision, roundedcyl_sphere) {
    // Rounded cylinder position and orientation fixed for all tests.
    // Aligned with X axis and shifted by its half-length in the X direction.
    real c_rad = 2.0;   // radius of skeleton cylinder
    real c_hlen = 1.5;  // half-length of skeleton cylinder
    real c_srad = 0.1;  // radius of sweeping sphere
    real3 c_pos(c_hlen, 0, 0);
    quaternion c_rot = FromChQuaternion(Q_from_AngY(CH_C_PI_2));

    ConvexShapeCustom* shapeC = new ConvexShapeCustom();
    shapeC->type = ChCollisionShape::Type::ROUNDEDCYL;
    shapeC->position = c_pos;
    shapeC->dimensions = real3(c_rad, c_hlen, c_rad);
    shapeC->radius = c_srad;
    shapeC->rotation = c_rot;

    // Sphere position changes for each test.
    real s_rad = 1.0;  // sphere radius

    ConvexShapeCustom* shapeS = new ConvexShapeCustom();
    shapeS->type = ChCollisionShape::Type::SPHERE;
    shapeS->dimensions = real3(s_rad, 0, 0);
    shapeS->radius = 0;
    shapeS->rotation = quaternion(1, 0, 0, 0);

    // Output quantities.
    real3 norm;
    real3 pt1;
    real3 pt2;
    real depth;
    real eff_rad;
    int nC;

    real oosqrt2 = sqrt(0.5);  // 1/sqrt(2)

    // sphere center inside cylinder
    {
        shapeS->position = real3(2.5, 1.5, 0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // cap interaction (separated)
    {
        shapeS->position = real3(4.5, 1.5, 0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // cap interaction (penetrated)
    {
        shapeS->position = real3(3.75, 1.5, 0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 1);
        Assert_near(norm, real3(1, 0, 0), precision);
        ASSERT_NEAR(depth, -0.35, precision);
        Assert_near(pt1, real3(3.1, 1.5, 0), precision);
        Assert_near(pt2, real3(2.75, 1.5, 0), precision);
        ASSERT_NEAR(eff_rad, s_rad, precision);
    }

    // side interaction (separated)
    {
        shapeS->position = real3(2.5, 3.5, 0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // side interaction (penetrated)
    {
        shapeS->position = real3(2.5, 2.5, 0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 1);
        Assert_near(norm, real3(0, 1, 0), precision);
        ASSERT_NEAR(depth, -0.6, precision);
        Assert_near(pt1, real3(2.5, 2.1, 0), precision);
        Assert_near(pt2, real3(2.5, 1.5, 0), precision);
        ASSERT_NEAR(eff_rad, s_rad * c_rad / (s_rad + c_rad), precision);
    }

    // edge interaction (separated)
    {
        shapeS->position = real3(4, 3, 0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 0);
    }

    // edge interaction (penetrated)
    {
        shapeS->position = real3(3.5, 2.5, 0);
        ASSERT_TRUE(ChNarrowphase::PRIMSCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC));
        ASSERT_EQ(nC, 1);
        Assert_near(norm, real3(oosqrt2, oosqrt2, 0), precision);
        ASSERT_NEAR(depth, -1.1 + oosqrt2, precision);
        Assert_near(pt1, real3(3.0 + 0.1 * oosqrt2, 2.0 + 0.1 * oosqrt2, 0), precision);
        Assert_near(pt2, real3(3.5 - oosqrt2, 2.5 - oosqrt2, 0), precision);
        ASSERT_NEAR(eff_rad, s_rad * c_srad / (s_rad + c_srad), precision);
    }

    delete shapeS;
    delete shapeC;
}

INSTANTIATE_TEST_SUITE_P(R, Collision, ::testing::Bool());
