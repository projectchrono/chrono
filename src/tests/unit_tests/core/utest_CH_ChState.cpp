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

using std::cout;
using std::endl;
using namespace chrono;

const double ABS_ERR_D = 1e-15;
const float ABS_ERR_F = 1e-6f;

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

    ASSERT_DEATH(s4 += s3, "^Assertion failed:");   // should be a run-time assertion failure
}

