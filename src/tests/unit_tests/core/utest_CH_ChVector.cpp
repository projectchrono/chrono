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
#include "chrono/core/ChVector3.h"

using namespace chrono;

const double ABS_ERR_D = 1e-15;
const float ABS_ERR_F = 1e-6f;

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
