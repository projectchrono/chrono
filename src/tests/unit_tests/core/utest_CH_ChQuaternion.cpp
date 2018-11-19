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

#include "gtest/gtest.h"
#include "chrono/core/ChQuaternion.h"

using namespace chrono;

static void TestEqualDouble(const ChQuaternion<double>& q1, const ChQuaternion<double>& q2) {
    ASSERT_DOUBLE_EQ(q1.e0(), q2.e0());
    ASSERT_DOUBLE_EQ(q1.e1(), q2.e1());
    ASSERT_DOUBLE_EQ(q1.e2(), q2.e2());
    ASSERT_DOUBLE_EQ(q1.e3(), q2.e3());
}

static void TestEqualFloat(const ChQuaternion<float>& q1, const ChQuaternion<float>& q2) {
    ASSERT_FLOAT_EQ(q1.e0(), q2.e0());
    ASSERT_FLOAT_EQ(q1.e1(), q2.e1());
    ASSERT_FLOAT_EQ(q1.e2(), q2.e2());
    ASSERT_FLOAT_EQ(q1.e3(), q2.e3());
}

TEST(ChQuaternionTest, multiply) {
    ChQuaternion<double> q1d(1.0, 0.2, 0.3, 0.1);
    ChQuaternion<double> q2d(0.7, 0.7, 0.0, 0.0);

    TestEqualDouble(q1d * q2d, Qcross(q1d, q2d));
    TestEqualDouble(q1d * q2d, q2d >> q1d);

    ChQuaternion<double> qd;
    qd.Cross(q1d, q2d);
    TestEqualDouble(q1d * q2d, qd);

    qd = q1d;
    qd *= q2d;
    TestEqualDouble(q1d * q2d, qd);

    qd = q1d;
    qd >>= q2d;
    TestEqualDouble(q2d * q1d, qd);

    ChQuaternion<float> q1f(1.0f, 0.2f, 0.3f, 0.1f);
    ChQuaternion<float> q2f(0.7f, 0.7f, 0.0f, 0.0f);

    TestEqualFloat(q1f * q2f, Qcross(q1f, q2f));
    TestEqualFloat(q1f * q2f, q2f >> q1f);

    ChQuaternion<float> qf;
    qf.Cross(q1f, q2f);
    TestEqualFloat(q1f * q2f, qf);

    qf = q1f;
    qf *= q2f;
    TestEqualFloat(q1f * q2f, qf);

    qf = q1f;
    qf >>= q2f;
    TestEqualFloat(q2f * q1f, qf);
}
