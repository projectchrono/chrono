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
#include "chrono/core/ChVector2.h"
#include "chrono/core/ChVector3.h"

using namespace chrono;

const double ABS_ERR_D = 1e-15;
const float ABS_ERR_F = 1e-6f;

TEST(ChVector, cast) {
    float s = 2.0f;
    {
        ChVector3f vec(1.5, 2.5, 3.5);
        ChVector3f res = s * vec;
        ASSERT_NEAR(res.x(), 3.0f, ABS_ERR_F);
        ASSERT_NEAR(res.y(), 5.0f, ABS_ERR_F);
        ASSERT_NEAR(res.z(), 7.0f, ABS_ERR_F);
    }
    {
        ChVector3d vec(1.5, 2.5, 3.5);
        ChVector3d res = s * vec;
        ASSERT_NEAR(res.x(), 3.0, ABS_ERR_D);
        ASSERT_NEAR(res.y(), 5.0, ABS_ERR_D);
        ASSERT_NEAR(res.z(), 7.0, ABS_ERR_D);
    }
}

TEST(ChVectorTest, scalar_mult_mixed) {
    // A double scalar times a float vector must not be routed through the integer-vector overload
    {
        ChVector3f vec(1.5, 2.5, 3.5);
        auto res = 2.0 * vec;
        static_assert(std::is_same<decltype(res), ChVector3f>::value, "double * ChVector3f must be a ChVector3f");
        ASSERT_NEAR(res.x(), 3.0f, ABS_ERR_F);
        ASSERT_NEAR(res.y(), 5.0f, ABS_ERR_F);
        ASSERT_NEAR(res.z(), 7.0f, ABS_ERR_F);
    }
    // A floating point scalar times an integer vector yields a double vector
    {
        ChVector3i vec(1, 2, 3);
        auto res = 2.5 * vec;
        static_assert(std::is_same<decltype(res), ChVector3d>::value, "double * ChVector3i must be a ChVector3d");
        ASSERT_NEAR(res.x(), 2.5, ABS_ERR_D);
        ASSERT_NEAR(res.y(), 5.0, ABS_ERR_D);
        ASSERT_NEAR(res.z(), 7.5, ABS_ERR_D);

        auto resf = 2.5f * vec;
        static_assert(std::is_same<decltype(resf), ChVector3d>::value, "float * ChVector3i must be a ChVector3d");
        ASSERT_NEAR(resf.x(), 2.5, ABS_ERR_D);
        ASSERT_NEAR(resf.y(), 5.0, ABS_ERR_D);
        ASSERT_NEAR(resf.z(), 7.5, ABS_ERR_D);
    }
    // An integer scalar times an integer vector remains an integer vector
    {
        ChVector3i vec(1, 2, 3);
        auto res = 2 * vec;
        static_assert(std::is_same<decltype(res), ChVector3i>::value, "int * ChVector3i must be a ChVector3i");
        ASSERT_EQ(res, ChVector3i(2, 4, 6));
    }
}

TEST(ChVectorTest, int_division) {
    {
        ChVector3i vec(4, -6, 9);
        ASSERT_EQ(vec / 2, ChVector3i(2, -3, 4));
        vec /= 2;
        ASSERT_EQ(vec, ChVector3i(2, -3, 4));
    }
    {
        ChVector2i vec(4, -6);
        ASSERT_EQ(vec / 2, ChVector2i(2, -3));
        vec /= 2;
        ASSERT_EQ(vec, ChVector2i(2, -3));
    }
}

TEST(ChVectorTest, no_implicit_scalar_conversion) {
    // Scalars must not implicitly convert to vectors (e.g. 'v + 1.0' or passing a scalar for a vector argument)
    static_assert(!std::is_convertible<double, ChVector3d>::value, "double must not implicitly convert to ChVector3d");
    static_assert(!std::is_convertible<float, ChVector3f>::value, "float must not implicitly convert to ChVector3f");
    static_assert(!std::is_convertible<int, ChVector3i>::value, "int must not implicitly convert to ChVector3i");
    static_assert(!std::is_convertible<double, ChVector2d>::value, "double must not implicitly convert to ChVector2d");
    static_assert(!std::is_convertible<int, ChVector2i>::value, "int must not implicitly convert to ChVector2i");

    // Explicit construction from a scalar is still available
    ASSERT_EQ(ChVector3d(2.0), ChVector3d(2.0, 2.0, 2.0));
    ASSERT_EQ(ChVector2d(2.0), ChVector2d(2.0, 2.0));
}

TEST(ChVectorTest, normalize) {
    ChVector3d ad(1.1, -2.2, 3.3);
    ASSERT_NEAR(ad.GetNormalized().Length(), 1.0, ABS_ERR_D);
    ASSERT_TRUE(ad.Normalize());
    ASSERT_NEAR(ad.Length(), 1.0, ABS_ERR_D);

    ChVector3d bd(0.0);
    ASSERT_FALSE(bd.Normalize());

    ChVector3f af(1.1f, -2.2f, 3.3f);
    ASSERT_NEAR(af.GetNormalized().Length(), 1.0f, ABS_ERR_F);
    ASSERT_TRUE(af.Normalize());
    ASSERT_NEAR(af.Length(), 1.0f, ABS_ERR_F);

    ChVector3f bf(0.0f);
    ASSERT_FALSE(bf.Normalize());
}

TEST(ChVectorTest, dot) {
    ChVector3d ad(1.1, -2.2, 3.3);
    ASSERT_NEAR(ad.Dot(ad), ad.Length2(), ABS_ERR_D);
    ASSERT_NEAR(ad.Dot(-ad), -ad.Length2(), ABS_ERR_D);
    ASSERT_NEAR(ad.Dot(ad.GetOrthogonalVector()), 0.0, ABS_ERR_D);

    ChVector3f af(1.1f, -2.2f, 3.3f);
    ASSERT_NEAR(af.Dot(af), af.Length2(), ABS_ERR_F);
    ASSERT_NEAR(af.Dot(-af), -af.Length2(), ABS_ERR_F);
    ASSERT_NEAR(af.Dot(af.GetOrthogonalVector()), 0.0f, ABS_ERR_F);
}

TEST(ChVectorTest, cross) {
    ChVector3d ad(1.1, -2.2, 3.3);
    ChVector3d bd(-0.5, 0.6, 0.7);
    auto cd = ad.Cross(bd);
    ASSERT_NEAR(cd.Dot(ad), 0.0, ABS_ERR_D);
    ASSERT_NEAR(cd.Dot(bd), 0.0, ABS_ERR_D);

    auto zd1 = ad.Cross(ad);
    ASSERT_NEAR(zd1.x(), 0.0, ABS_ERR_D);
    ASSERT_NEAR(zd1.y(), 0.0, ABS_ERR_D);
    ASSERT_NEAR(zd1.z(), 0.0, ABS_ERR_D);

    auto zd2 = ad.Cross(-ad);
    ASSERT_NEAR(zd2.x(), 0.0, ABS_ERR_D);
    ASSERT_NEAR(zd2.y(), 0.0, ABS_ERR_D);
    ASSERT_NEAR(zd2.z(), 0.0, ABS_ERR_D);

    auto pd = ad.GetOrthogonalVector();
    ASSERT_NEAR(ad.Cross(pd).Length(), ad.Length() * pd.Length(), ABS_ERR_D);

    ChVector3f af(1.1f, -2.2f, 3.3f);
    ChVector3f bf(-0.5f, 0.6f, 0.7f);
    auto cf = af.Cross(bf);
    ASSERT_NEAR(cf.Dot(af), 0.0f, ABS_ERR_F);
    ASSERT_NEAR(cf.Dot(bf), 0.0f, ABS_ERR_F);

    auto zf1 = af.Cross(af);
    ASSERT_NEAR(zf1.x(), 0.0f, ABS_ERR_F);
    ASSERT_NEAR(zf1.y(), 0.0f, ABS_ERR_F);
    ASSERT_NEAR(zf1.z(), 0.0f, ABS_ERR_F);

    auto zf2 = af.Cross(-af);
    ASSERT_NEAR(zf2.x(), 0.0f, ABS_ERR_F);
    ASSERT_NEAR(zf2.y(), 0.0f, ABS_ERR_F);
    ASSERT_NEAR(zf2.z(), 0.0f, ABS_ERR_F);

    auto pf = af.GetOrthogonalVector();
    ASSERT_NEAR(af.Cross(pf).Length(), af.Length() * pf.Length(), ABS_ERR_F);
}
