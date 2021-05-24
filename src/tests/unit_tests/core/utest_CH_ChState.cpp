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
// Test of operations with 3d vectors
//
// =============================================================================

#include "gtest/gtest.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/core/ChCoordsys.h"

using std::cout;
using std::endl;
using namespace chrono;

TEST(ChStateTest, create_assign) {
    ChState s1;
    s1.resize(5);
    s1(0) = 1;
    s1(1) = 2;
    s1(2) = 3;
    s1(3) = 4;
    s1(4) = 5;

    ChState s2(5, nullptr);
    s2(0) = 1;
    s2(1) = 2;
    s2(2) = 3;
    s2(3) = 4;
    s2(4) = 5;

    ChVectorDynamic<> v(5);
    v << 1, 2, 3, 4, 5;
    ChState s3(v, nullptr);

    ChState s4(s3);

    cout << "s1\n" << s1.transpose() << endl;
    cout << "s2\n" << s2.transpose() << endl;
    cout << "s3\n" << s3.transpose() << endl;
    cout << "s4\n" << s4.transpose() << endl;

    ASSERT_TRUE(s1 == s2);
    ASSERT_TRUE(s1 == s3);
    ASSERT_TRUE(s1 == s4);

    cout << "s1 + s3\n" << (s1 + s3).transpose() << endl;
    cout << "s1 + v\n" << (s1 + v).transpose() << endl;
    ASSERT_TRUE(s1 + s3 == s1 + v);

    s4.setZero(6, nullptr);
    cout << "s4 = 0\n" << s4.transpose() << endl;
    ASSERT_DOUBLE_EQ(s4.norm(), 0.0);

    s3 += s1;
    cout << "s3 += s1\n" << s3.transpose() << endl;

    s3 *= 2;
    cout << "s3 *= 2\n" << s3.transpose() << endl;

    cout << "-s3\n" << (-s3).transpose() << endl;

    ////ASSERT_DEATH(s4 += s3, "^Assertion failed:");   // should be a run-time assertion failure (only valid in 'Debug' mode)
}

// Test inter-operability with ChVector, ChQuaternion, and ChCoordsys
TEST(ChStateTest, interop) {

    ChVector<> v1(1, 2, 3);
    ChQuaternion<> q1(10, 20, 30, 40);
    ChCoordsys<> cs1(v1, q1);

    {
        ChState S;
        S.setZero(12, nullptr);

        S.segment(2, 3) = v1.eigen();
        cout << "v -> S\n" << S.transpose() << endl;
        ChVector<> v2(S.segment(2, 3));
        cout << "S -> v (constructor)\n" << v2 << endl;
        ChVector<> v3(0);
        v3 = S.segment(2, 3);
        cout << "S -> v (assignment)\n" << v3 << endl;

        ASSERT_TRUE(v1.Equals(v2));
        ASSERT_TRUE(v1.Equals(v3));

        S.segment(5, 4) = q1.eigen();
        cout << "q -> S\n" << S.transpose() << endl;
        ChQuaternion<> q2(S.segment(5, 4));
        cout << "S -> q (constructor)\n" << q2 << endl;
        ChQuaternion<> q3(0, 0, 0, 0);
        q3 = S.segment(5, 4);
        cout << "S -> q (assignment)\n" << q3 << endl;

        ASSERT_TRUE(q1.Equals(q2));
        ASSERT_TRUE(q1.Equals(q3));

        S.setZero(12, nullptr);

        S.segment(1, 3) = cs1.pos.eigen();
        S.segment(4, 4) = cs1.rot.eigen();
        cout << "cs -> S\n" << S.transpose() << endl;
        ChCoordsys<> cs2(S.segment(1, 7));
        cout << "S -> cs (constructor)\n" << cs2 << endl;
        ChCoordsys<> cs3(VNULL, QNULL);
        cs3 = S.segment(1, 7);
        cout << "S -> cs (assignment)\n" << cs3 << endl;

        ASSERT_TRUE(cs1.Equals(cs2));
        ASSERT_TRUE(cs1.Equals(cs3));
    }

    {
        ChStateDelta SD;
        SD.setZero(12, nullptr);

        SD.segment(2, 3) = v1.eigen();
        ChVector<> v2(SD.segment(2, 3));
        ChVector<> v3(0);
        v3 = SD.segment(2, 3);

        ASSERT_TRUE(v1.Equals(v2));
        ASSERT_TRUE(v1.Equals(v3));

        SD.segment(5, 4) = q1.eigen();
        ChQuaternion<> q2(SD.segment(5, 4));
        ChQuaternion<> q3(0, 0, 0, 0);
        q3 = SD.segment(5, 4);

        ASSERT_TRUE(q1.Equals(q2));
        ASSERT_TRUE(q1.Equals(q3));

        SD.setZero(12, nullptr);

        SD.segment(1, 3) = cs1.pos.eigen();
        SD.segment(4, 4) = cs1.rot.eigen();
        ChCoordsys<> cs2(SD.segment(1, 7));
        ChCoordsys<> cs3(VNULL, QNULL);
        cs3 = SD.segment(1, 7);

        ASSERT_TRUE(cs1.Equals(cs2));
        ASSERT_TRUE(cs1.Equals(cs3));
    }

    {
        ChVectorDynamic<> V;
        V.setZero(12);

        V.segment(2, 3) = v1.eigen();
        ChVector<> v2(V.segment(2, 3));
        ChVector<> v3(0);
        v3 = V.segment(2, 3);

        ASSERT_TRUE(v1.Equals(v2));
        ASSERT_TRUE(v1.Equals(v3));

        V.segment(5, 4) = q1.eigen();
        ChQuaternion<> q2(V.segment(5, 4));
        ChQuaternion<> q3(0, 0, 0, 0);
        q3 = V.segment(5, 4);

        ASSERT_TRUE(q1.Equals(q2));
        ASSERT_TRUE(q1.Equals(q3));

        V.setZero(12);

        V.segment(1, 3) = cs1.pos.eigen();
        V.segment(4, 4) = cs1.rot.eigen();
        ChCoordsys<> cs2(V.segment(1, 7));
        ChCoordsys<> cs3(VNULL, QNULL);
        cs3 = V.segment(1, 7);

        ASSERT_TRUE(cs1.Equals(cs2));
        ASSERT_TRUE(cs1.Equals(cs3));
    }
}
