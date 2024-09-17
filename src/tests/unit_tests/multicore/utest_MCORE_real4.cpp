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
// Chrono::Multicore unit test for real class
// =============================================================================

#include "unit_testing.h"

#ifdef CHRONO_MULTICORE_USE_DOUBLE
const double precision = 1e-10;
#else
const float precision = 1e-6f;
#endif

TEST(real4, functions) {
    {
        // quaternion ~
        quaternion b(11 / 2.0, -2, -3, -2);
        quaternion c = ~b;
        ASSERT_NEAR(c.w, 5.5, precision);
        ASSERT_NEAR(c.x, 2.0, precision);
        ASSERT_NEAR(c.y, 3.0, precision);
        ASSERT_NEAR(c.z, 2.0, precision);
    }
    {
        // quaternion dot
        quaternion a(11, -2, 0, -2);
        real b = Dot(a);
        real c = 11 * 11 + -2 * -2 + 0 * 0 + -2 * -2;
        ASSERT_EQ(b, c);
    }
    {
        // quaternion inverse
        quaternion a(11, -2, 0, -2);
        quaternion b = Inv(a);
        ASSERT_EQ(b.w, 11.0 / 129.0);
        ASSERT_EQ(b.x, 2.0 / 129.0);
        ASSERT_EQ(b.y, 0.0);
        ASSERT_EQ(b.z, 2.0 / 129.0);
    }

    {
        // quaternion normalize
        quaternion a(11, -2, 0, -2);
        quaternion b = Normalize(a);
        ASSERT_NEAR(b.w, 11.0 / std::sqrt(129.0), precision);
        ASSERT_NEAR(b.x, -2.0 / std::sqrt(129.0), precision);
        ASSERT_NEAR(b.y, 0.0, precision);
        ASSERT_NEAR(b.z, -2.0 / std::sqrt(129.0), precision);
    }

    {
        // quaternion multiply
        quaternion a(11 / 2.0, -2, -3, -2);
        quaternion b(11, -2, 0, -2);
        quaternion c = Mult(a, b);
        ASSERT_NEAR(c.w, 105 / 2.0, precision);
        ASSERT_NEAR(c.x, -27.0, precision);
        ASSERT_NEAR(c.y, -33.0, precision);
        ASSERT_NEAR(c.z, -39.0, precision);
    }

    {
        // quaternion multiply
        quaternion R1 = Normalize(quaternion(rand(), rand(), rand(), rand()));
        quaternion R2 = Normalize(quaternion(rand(), rand(), rand(), rand()));

        quaternion Res1 = Mult(R1, R2);
        ChQuaternion<real> Res2;
        Res2.Cross(ToChQuaternion(R1), ToChQuaternion(R2));
        Assert_near(Res1, FromChQuaternion(Res2), precision);
    }

    {
        // quaternion rotate
        real3 a(1.0, 2.0, -3.0);
        quaternion b(2.0, -2, -2, -2);
        b = Normalize(b);

        real3 c = Rotate(a, b);
        ASSERT_NEAR(c.x, 2, precision);
        ASSERT_NEAR(c.y, -3, precision);
        ASSERT_NEAR(c.z, 1, precision);
    }

    {
        // quaternion rotate
        real3 a(1.0, 2.0, -3.0);
        quaternion b(2.0, -2, -2, -2);
        b = Normalize(b);
        real3 c = RotateT(a, b);
        ASSERT_NEAR(c.x, -3, precision);
        ASSERT_NEAR(c.y, 1, precision);
        ASSERT_NEAR(c.z, 2, precision);
    }

    {
        // real4 rotate
        real3 V(1.0, 2.0, -3.0);
        quaternion R1 = Normalize(quaternion(rand(), rand(), rand(), rand()));
        real3 Res1a = Rotate(V, R1);
        ChQuaternion<real> R2 = ToChQuaternion(R1);

        ChVector3<real> Res2 = R2.Rotate(ToChVector(V));
        Assert_near(Res1a, FromChVector(Res2), precision);
    }

    {
        // quaternion conjugate

        quaternion R1 = Normalize(quaternion(rand(), rand(), rand(), rand()));
        quaternion Res1 = ~R1;
        ChQuaternion<real> R2 = ToChQuaternion(R1);
        R2.Conjugate();
        ChQuaternion<real> Res2 = R2;
        Assert_near(Res1, FromChQuaternion(Res2), precision);
    }

    {
        // quaternion AbsRotate
        quaternion R1 = Normalize(quaternion(rand(), rand(), rand(), rand()));
        real3 V(1.0, 2.0, 3.0);
        real3 result_1 = AbsRotate(R1, V);

        Mat33 R2(R1);
        R2 = Abs(R2);
        real3 result_2 = R2 * V;

        Assert_near(result_1, result_2, precision * 2);
    }

    {
        // converting from parent to local frame
        quaternion R = Normalize(quaternion(rand(), rand(), rand(), rand()));
        real3 pos = Normalize(real3(rand(), rand(), rand()));
        real3 p_loc = Normalize(real3(rand(), rand(), rand()));

        real3 p_abs = pos + Rotate(p_loc, R);

        Assert_near(Rotate(p_abs - pos, Inv(R)), p_loc, precision);
        Assert_near(RotateT(p_abs - pos, R), p_loc, precision);
    }
}
