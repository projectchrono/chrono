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
// Authors: Hammad Mazhar
// =============================================================================
//
// ChronoParallel unit test for real class
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>

#include "unit_testing.h"

#include "chrono_parallel/math/real3.h"

#ifdef CHRONO_PARALLEL_USE_DOUBLE
const double precision = 1e-10;
#else
const float precision = 1e-6f;
#endif

int main(int argc, char* argv[]) {
    // =============================================================================
    {
        std::cout << "zero constructor\n";
        real3 zero(0);
        WeakEqual(zero.x, 0.0, precision);
        WeakEqual(zero.y, 0.0, precision);
        WeakEqual(zero.z, 0.0, precision);
    }

    {
        std::cout << "1 float constructor\n";
        real3 value(1.5);
        WeakEqual(value.x, 1.5, precision);
        WeakEqual(value.y, 1.5, precision);
        WeakEqual(value.z, 1.5, precision);
    }

    {
        std::cout << "3 float constructor\n";
        real3 value(1.5, 2.6, 3.7);
        WeakEqual(value.x, 1.5, precision);
        WeakEqual(value.y, 2.6, precision);
        WeakEqual(value.z, 3.7, precision);
    }
    // =============================================================================
    {
        std::cout << "float 3 add\n";
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        real3 c = a + b;
        WeakEqual(c.x, 4.0, precision);
        WeakEqual(c.y, 4.0, precision);
        WeakEqual(c.z, 4.0, precision);
    }

    {
        std::cout << "float 3 subtract\n";
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        real3 c = a - b;
        WeakEqual(c.x, -2.0, precision);
        WeakEqual(c.y, 0.0, precision);
        WeakEqual(c.z, 2.0, precision);
    }

    {
        std::cout << "float 3 multiply\n";
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        real3 c = a * b;
        WeakEqual(c.x, 3.0, precision);
        WeakEqual(c.y, 4.0, precision);
        WeakEqual(c.z, 3.0, precision);
    }

    {
        std::cout << "float 3 divide\n";
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        real3 c = a / b;
        WeakEqual(c.x, 1.0 / 3.0, precision);
        WeakEqual(c.y, 2.0 / 2.0, precision);
        WeakEqual(c.z, 3.0 / 1.0, precision);
    }
    {
        std::cout << "float 3 negate\n";
        real3 a(1.0, 2.0, 3.0);
        real3 c = -a;
        WeakEqual(c.x, -1.0, precision);
        WeakEqual(c.y, -2.0, precision);
        WeakEqual(c.z, -3.0, precision);
    }
    // =============================================================================
    {
        std::cout << "float 3 add\n";
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        a += b;
        WeakEqual(a.x, 4.0, precision);
        WeakEqual(a.y, 4.0, precision);
        WeakEqual(a.z, 4.0, precision);
    }

    {
        std::cout << "float 3 subtract\n";
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        a -= b;
        WeakEqual(a.x, -2.0, precision);
        WeakEqual(a.y, 0.0, precision);
        WeakEqual(a.z, 2.0, precision);
    }

    {
        std::cout << "float 3 multiply\n";
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        a *= b;
        WeakEqual(a.x, 3.0, precision);
        WeakEqual(a.y, 4.0, precision);
        WeakEqual(a.z, 3.0, precision);
    }

    {
        std::cout << "float 3 divide\n";
        real3 a(1.0, 2.0, 3.0);
        real3 b(3.0, 2.0, 1.0);
        a /= b;
        WeakEqual(a.x, 1.0 / 3.0, precision);
        WeakEqual(a.y, 2.0 / 2.0, precision);
        WeakEqual(a.z, 3.0 / 1.0, precision);
    }
    // =============================================================================

    {
        std::cout << "float 3 add\n";
        real3 a(1.0, 2.0, 3.0);
        real b(2.0);
        real3 c = a + b;
        WeakEqual(c.x, 3.0, precision);
        WeakEqual(c.y, 4.0, precision);
        WeakEqual(c.z, 5.0, precision);
    }

    {
        std::cout << "float 3 subtract\n";
        real3 a(1.0, 2.0, 3.0);
        real b(2.0);
        real3 c = a - b;
        WeakEqual(c.x, -1.0, precision);
        WeakEqual(c.y, 0.0, precision);
        WeakEqual(c.z, 1.0, precision);
    }

    {
        std::cout << "float 3 multiply\n";
        real3 a(1.0, 2.0, 3.0);
        real b(2.0);
        real3 c = a * b;
        WeakEqual(c.x, 2.0, precision);
        WeakEqual(c.y, 4.0, precision);
        WeakEqual(c.z, 6.0, precision);
    }

    {
        std::cout << "float 3 divide\n";
        real3 a(1.0, 2.0, 3.0);
        real b(2.0);
        real3 c = a / b;
        WeakEqual(c.x, 1.0 / 2.0, precision);
        WeakEqual(c.y, 2.0 / 2.0, precision);
        WeakEqual(c.z, 3.0 / 2.0, precision);
    }
    // =============================================================================

    {
        std::cout << "float 3 add\n";
        real3 a(1.0, 2.0, 3.0);
        real b(2.0);
        a += b;
        WeakEqual(a.x, 3.0, precision);
        WeakEqual(a.y, 4.0, precision);
        WeakEqual(a.z, 5.0, precision);
    }

    {
        std::cout << "float 3 subtract\n";
        real3 a(1.0, 2.0, 3.0);
        real b(2.0);
        a -= b;
        WeakEqual(a.x, -1.0, precision);
        WeakEqual(a.y, 0.0, precision);
        WeakEqual(a.z, 1.0, precision);
    }

    {
        std::cout << "float 3 multiply\n";
        real3 a(1.0, 2.0, 3.0);
        real b(2.0);
        a *= b;
        WeakEqual(a.x, 2.0, precision);
        WeakEqual(a.y, 4.0, precision);
        WeakEqual(a.z, 6.0, precision);
    }

    {
        std::cout << "float 3 divide\n";
        real3 a(1.0, 2.0, 3.0);
        real b(2.0);
        a /= b;
        WeakEqual(a.x, 1.0 / 2.0, precision);
        WeakEqual(a.y, 2.0 / 2.0, precision);
        WeakEqual(a.z, 3.0 / 2.0, precision);
    }
    // =============================================================================

    {
        std::cout << "float 3 dot\n";
        real3 a(1.0, 2.0, 3.0);
        real3 b(2.0, 1.0, 3.0);
        real c = Dot(a, b);
        WeakEqual(c, 13.0, precision);
    }
    {
        std::cout << "float 3 dot\n";
        real3 a(0, -2.0, 0);
        real3 b(0, -2.0, 0);
        real c = Dot(a, b);
        WeakEqual(c, 4.0, precision);
    }
    {
        std::cout << "float 3 dot\n";
        real3 a(0, -2, 0);
        real c = Dot(a);
        WeakEqual(c, 4.0, precision);
    }
    {
        std::cout << "float 3 cross\n";
        real3 a(1.0, 2.0, 3.0);
        real3 b(2.0, 1.0, 3.0);
        real3 c = Cross(a, b);
        WeakEqual(c.x, 3.0, precision);
        WeakEqual(c.y, 3.0, precision);
        WeakEqual(c.z, -3.0, precision);
    }
    {
        std::cout << "float 3 cross\n";
        real3 a = Normalize(real3(rand(), rand(), rand()));
        real3 b = Normalize(real3(rand(), rand(), rand()));
        real3 ans1 = Cross(a, b);
        ChVector<real> ans2;
        ans2.Cross(ToChVector(a), ToChVector(b));
        WeakEqual(ans1, ToReal3(ans2), precision);
    }

    {
        std::cout << "float 3 length\n";
        real3 a(1.0, 2.0, -3.0);
        real c = Length(a);
        WeakEqual(c, sqrt(14.0), precision);
    }
    {
        std::cout << "float 3 normalize\n";
        real3 a(1.0, 2.0, -3.0);
        real3 c = Normalize(a);
        WeakEqual(c.x, 1.0 / sqrt(14.0), precision);
        WeakEqual(c.y, 2.0 / sqrt(14.0), precision);
        WeakEqual(c.z, -3.0 / sqrt(14.0), precision);
    }

    {
        std::cout << "float 3 ==\n";
        real3 a(1.0, 2.0, -3.0);
        real3 c = Normalize(a);
        bool res = (a == c);
        StrictEqual(res, 0);
        res = (a == a);
        StrictEqual(res, 1);
    }

    {
        std::cout << "float 3 abs\n";
        real3 a(-1.0, 2.0, -3.0);
        real3 c = Abs(a);
        WeakEqual(c[0], 1.0, precision);
        WeakEqual(c[1], 2.0, precision);
        WeakEqual(c[2], 3.0, precision);
    }
    {
        std::cout << "float 3 Sign\n";
        real3 a(-1.0, 2.0, 0);
        real3 c = Sign(a);
        WeakEqual(c[0], -1, precision);
        WeakEqual(c[1], 1, precision);
        WeakEqual(c[2], 0, precision);
    }
    {
        std::cout << "float 3 Clamp\n";
        real3 a(-10.0, 3.0, 5);
        real3 mi(-1.0, -1.0, -1.0);
        real3 ma(2.0, 2.0, 4.0);
        real3 c = Clamp(a, mi, ma);
        WeakEqual(c[0], -1.0, precision);
        WeakEqual(c[1], 2.0, precision);
        WeakEqual(c[2], 4.0, precision);
    }
    {
        std::cout << "float 3 Min\n";
        real3 a(10.0, -10.0, 5.0);
        real res = Min(a);
        WeakEqual(res, -10.0, precision);
    }
	{
		std::cout << "float 3 Min\n";
		real3 a(3.0, 1.0, 5);
		real res = Min(a);
		WeakEqual(res, 1.0, precision);
	}
    {
        std::cout << "float 3 Max\n";
        real3 a(-10.0, 3.0, 5);
        real res = Max(a);
        WeakEqual(res, 5.0, precision);
    }
	{
		std::cout << "float 3 Max\n";
		real3 a(-10.0, -3.0, -5);
		real res = Max(a);
		WeakEqual(res, -3.0, precision);
	}
    {
        std::cout << "float 3 Round\n";
        real3 a(-3.2, 3.6, 5.4);
        real3 c = Round(a);
        WeakEqual(c[0], -3, precision);
        WeakEqual(c[1], 4, precision);
        WeakEqual(c[2], 5, precision);
    }
    {
        std::cout << "float 3 IsZero\n";
        real3 a(-3.2, 0, 5.4);
        bool c = IsZero(a);
        StrictEqual(c, false);
        a = real3(0, 0, 0);
        c = IsZero(a);
        StrictEqual(c, true);
    }

	{
		std::cout << "float 3 add, divite, dot product\n";
		real3 a(1.0, 2.0, 3.0);
		real3 b(3.0, 2.0, 1.0);
		real3 c(4.0, 5.0, 8.0);
		a = (a+b)/c;
		real d = Dot(a,b);

		WeakEqual(d, 5.1, precision);

	}
    return 0;
}
