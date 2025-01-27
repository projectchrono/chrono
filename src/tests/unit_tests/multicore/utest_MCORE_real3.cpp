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
// Chrono::Multicore unit test for real3 class
// =============================================================================

#include "../ut_utils.h"

#ifdef CHRONO_MULTICORE_USE_DOUBLE
const double precision = 1e-10;
#else
const float precision = 1e-6f;
#endif

TEST(real3, constructors) {
    {
        // zero constructor
        real3 zero(0);
        ASSERT_EQ(zero.x, 0.0);
        ASSERT_EQ(zero.y, 0.0);
        ASSERT_EQ(zero.z, 0.0);
    }
    {
        // 1 float constructor
        real3 value(1.5);
        ASSERT_EQ(value.x, 1.5);
        ASSERT_EQ(value.y, 1.5);
        ASSERT_EQ(value.z, 1.5);
    }
    {
        // 3 float constructor
        real3 value(1.5, 2.6, 3.7);
        ASSERT_EQ(value.x, 1.5);
        ASSERT_EQ(value.y, 2.6);
        ASSERT_EQ(value.z, 3.7);
    }
}

TEST(real3, operators) {
    {
        // float 3 add
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        real3 c = a + b;
        ASSERT_NEAR(c.x, 4.0, precision);
        ASSERT_NEAR(c.y, 4.0, precision);
        ASSERT_NEAR(c.z, 4.0, precision);
    }
    {
        // float 3 subtract
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        real3 c = a - b;
        ASSERT_NEAR(c.x, -2.0, precision);
        ASSERT_NEAR(c.y, 0.0, precision);
        ASSERT_NEAR(c.z, 2.0, precision);
    }
    {
        // float 3 multiply
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        real3 c = a * b;
        ASSERT_NEAR(c.x, 3.0, precision);
        ASSERT_NEAR(c.y, 4.0, precision);
        ASSERT_NEAR(c.z, 3.0, precision);
    }
    {
        // float 3 divide
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        real3 c = a / b;
        ASSERT_NEAR(c.x, 1.0 / 3.0, precision);
        ASSERT_NEAR(c.y, 2.0 / 2.0, precision);
        ASSERT_NEAR(c.z, 3.0 / 1.0, precision);
    }
    {
        // float 3 negate
        real3 a(1.0, 2.0, 3.0);
        real3 c = -a;
        ASSERT_NEAR(c.x, -1.0, precision);
        ASSERT_NEAR(c.y, -2.0, precision);
        ASSERT_NEAR(c.z, -3.0, precision);
    }
    // =============================================================================
    {
        // float 3 add
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        a += b;
        ASSERT_NEAR(a.x, 4.0, precision);
        ASSERT_NEAR(a.y, 4.0, precision);
        ASSERT_NEAR(a.z, 4.0, precision);
    }
    {
        // float 3 subtract
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        a -= b;
        ASSERT_NEAR(a.x, -2.0, precision);
        ASSERT_NEAR(a.y, 0.0, precision);
        ASSERT_NEAR(a.z, 2.0, precision);
    }
    {
        // float 3 multiply
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        a *= b;
        ASSERT_NEAR(a.x, 3.0, precision);
        ASSERT_NEAR(a.y, 4.0, precision);
        ASSERT_NEAR(a.z, 3.0, precision);
    }
    {
        // float 3 divide
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        a /= b;
        ASSERT_NEAR(a.x, 1.0 / 3.0, precision);
        ASSERT_NEAR(a.y, 2.0 / 2.0, precision);
        ASSERT_NEAR(a.z, 3.0 / 1.0, precision);
    }
    // =============================================================================
    {
        // float 3 add
        real3 a(1.0, 2.0, 3.0);
        real b(2.0);
        real3 c = a + b;
        ASSERT_NEAR(c.x, 3.0, precision);
        ASSERT_NEAR(c.y, 4.0, precision);
        ASSERT_NEAR(c.z, 5.0, precision);
    }
    {
        // float 3 subtract
        real3 a(1.0, 2.0, 3.0);
        real b(2.0);
        real3 c = a - b;
        ASSERT_NEAR(c.x, -1.0, precision);
        ASSERT_NEAR(c.y, 0.0, precision);
        ASSERT_NEAR(c.z, 1.0, precision);
    }
    {
        // float 3 multiply
        real3 a(1.0, 2.0, 3.0);
        real b(2.0);
        real3 c = a * b;
        ASSERT_NEAR(c.x, 2.0, precision);
        ASSERT_NEAR(c.y, 4.0, precision);
        ASSERT_NEAR(c.z, 6.0, precision);
    }
    {
        // float 3 divide
        real3 a(1.0, 2.0, 3.0);
        real b(2.0);
        real3 c = a / b;
        ASSERT_NEAR(c.x, 1.0 / 2.0, precision);
        ASSERT_NEAR(c.y, 2.0 / 2.0, precision);
        ASSERT_NEAR(c.z, 3.0 / 2.0, precision);
    }
    // =============================================================================
    {
        // float 3 add
        real3 a(1.0, 2.0, 3.0);
        real b(2.0);
        a += b;
        ASSERT_NEAR(a.x, 3.0, precision);
        ASSERT_NEAR(a.y, 4.0, precision);
        ASSERT_NEAR(a.z, 5.0, precision);
    }
    {
        // float 3 subtract
        real3 a(1.0, 2.0, 3.0);
        real b(2.0);
        a -= b;
        ASSERT_NEAR(a.x, -1.0, precision);
        ASSERT_NEAR(a.y, 0.0, precision);
        ASSERT_NEAR(a.z, 1.0, precision);
    }
    {
        // float 3 multiply
        real3 a(1.0, 2.0, 3.0);
        real b(2.0);
        a *= b;
        ASSERT_NEAR(a.x, 2.0, precision);
        ASSERT_NEAR(a.y, 4.0, precision);
        ASSERT_NEAR(a.z, 6.0, precision);
    }
    {
        // float 3 divide
        real3 a(1.0, 2.0, 3.0);
        real b(2.0);
        a /= b;
        ASSERT_NEAR(a.x, 1.0 / 2.0, precision);
        ASSERT_NEAR(a.y, 2.0 / 2.0, precision);
        ASSERT_NEAR(a.z, 3.0 / 2.0, precision);
    }
}

TEST(real3, functions) {
    {
        // float 3 dot
        real3 a(1.0, 2.0, 3.0);
        real3 b(2.0, 1.0, 3.0);
        real c = Dot(a, b);
        ASSERT_NEAR(c, 13.0, precision);
    }
    {
        // float 3 dot
        real3 a(0, -2.0, 0);
        real3 b(0, -2.0, 0);
        real c = Dot(a, b);
        ASSERT_NEAR(c, 4.0, precision);
    }
    {
        // float 3 dot
        real3 a(0, -2, 0);
        real c = Dot(a);
        ASSERT_NEAR(c, 4.0, precision);
    }
    {
        // float 3 cross
        real3 a(1.0, 2.0, 3.0);
        real3 b(2.0, 1.0, 3.0);
        real3 c = Cross(a, b);
        ASSERT_NEAR(c.x, 3.0, precision);
        ASSERT_NEAR(c.y, 3.0, precision);
        ASSERT_NEAR(c.z, -3.0, precision);
    }
    {
        // float 3 cross
        real3 a = Normalize(real3(rand(), rand(), rand()));
        real3 b = Normalize(real3(rand(), rand(), rand()));
        real3 ans1 = Cross(a, b);
        ChVector3<real> ans2;
        ans2.Cross(ToChVector(a), ToChVector(b));
        Assert_near(ans1, FromChVector(ans2), precision);
    }
    {
        // float 3 length
        real3 a(1.0, 2.0, -3.0);
        real c = Length(a);
        ASSERT_NEAR(c, std::sqrt(14.0), precision);
    }
    {
        // float 3 normalize
        real3 a(1.0, 2.0, -3.0);
        real3 c = Normalize(a);
        ASSERT_NEAR(c.x, 1.0 / std::sqrt(14.0), precision);
        ASSERT_NEAR(c.y, 2.0 / std::sqrt(14.0), precision);
        ASSERT_NEAR(c.z, -3.0 / std::sqrt(14.0), precision);
    }
    {
        // float 3 ==
        real3 a(1.0, 2.0, -3.0);
        real3 c = Normalize(a);
        ASSERT_FALSE(a == c);
        ASSERT_TRUE(a == a);
    }
    {
        // float 3 abs
        real3 a(-1.0, 2.0, -3.0);
        real3 c = Abs(a);
        ASSERT_NEAR(c[0], 1.0, precision);
        ASSERT_NEAR(c[1], 2.0, precision);
        ASSERT_NEAR(c[2], 3.0, precision);
    }
    {
        // float 3 Sign
        real3 a(-1.0, 2.0, 0);
        real3 c = Sign(a);
        ASSERT_NEAR(c[0], -1, precision);
        ASSERT_NEAR(c[1], 1, precision);
        ASSERT_NEAR(c[2], 0, precision);
    }
    {
        // float 3 Clamp
        real3 a(-10.0, 3.0, 5);
        real3 mi(-1.0, -1.0, -1.0);
        real3 ma(2.0, 2.0, 4.0);
        real3 c = Clamp(a, mi, ma);
        ASSERT_NEAR(c[0], -1.0, precision);
        ASSERT_NEAR(c[1], 2.0, precision);
        ASSERT_NEAR(c[2], 4.0, precision);
    }
    {
        // float 3 Min
        real3 a(10.0, -10.0, 5.0);
        real res = Min(a);
        ASSERT_NEAR(res, -10.0, precision);
    }
    {
        // float 3 Min
        real3 a(3.0, 1.0, 5);
        real res = Min(a);
        ASSERT_NEAR(res, 1.0, precision);
    }
    {
        // float 3 Max
        real3 a(-10.0, 3.0, 5);
        real res = Max(a);
        ASSERT_NEAR(res, 5.0, precision);
    }
    {
        // float 3 Max
        real3 a(-10.0, -3.0, -5);
        real res = Max(a);
        ASSERT_NEAR(res, -3.0, precision);
    }
    {
        // float 3 Round
        real3 a(-3.2, 3.6, 5.4);
        real3 c = Round(a);
        ASSERT_NEAR(c[0], -3, precision);
        ASSERT_NEAR(c[1], 4, precision);
        ASSERT_NEAR(c[2], 5, precision);
    }
    {
        // float 3 IsZero
        real3 a(-3.2, 0, 5.4);
        ASSERT_FALSE(IsZero(a));
        a = real3(0, 0, 0);
        ASSERT_TRUE(IsZero(a));
    }
    {
        // float 3 add, divite, dot product
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        real3 c(4.0, 5.0, 8.0);
        a = (a + b) / c;
        real d = Dot(a, b);

        ASSERT_NEAR(d, 5.1, precision);
    }
}
