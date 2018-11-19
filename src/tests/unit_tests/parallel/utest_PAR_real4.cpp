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

#include "chrono_parallel/math/real4.h"
#include "chrono_parallel/math/matrix.h"

#ifdef CHRONO_PARALLEL_USE_DOUBLE
const double precision = 1e-10;
#else
const float precision = 1e-6f;
#endif

int main(int argc, char* argv[]) {
    // =============================================================================

    {
        std::cout << "quaternion ~\n";
        quaternion b(11 / 2.0, -2, -3, -2);
        quaternion c = ~b;
        WeakEqual(c.w, 5.5, precision);
        WeakEqual(c.x, 2.0, precision);
        WeakEqual(c.y, 3.0, precision);
        WeakEqual(c.z, 2.0, precision);
    }
    {
        std::cout << "quaternion dot\n";
        quaternion a(11, -2, 0, -2);
        real b = Dot(a);
        real c = 11 * 11 + -2 * -2 + 0 * 0 + -2 * -2;
        StrictEqual(b, c);
    }

    {
        std::cout << "quaternion inverse\n";
        quaternion a(11, -2, 0, -2);
        quaternion b = Inv(a);
        StrictEqual(b.w, 11.0 / 129.0);
        StrictEqual(b.x, 2.0 / 129.0);
        StrictEqual(b.y, 0.0);
        StrictEqual(b.z, 2.0 / 129.0);
    }
    // =============================================================================
    {
        std::cout << "quaternion normalize\n";
        quaternion a(11, -2, 0, -2);
        quaternion b = Normalize(a);
        WeakEqual(b.w, 11.0 / sqrt(129.0), precision);
        WeakEqual(b.x, -2.0 / sqrt(129.0), precision);
        WeakEqual(b.y, 0.0, precision);
        WeakEqual(b.z, -2.0 / sqrt(129.0), precision);
    }
    // =============================================================================
    {
        std::cout << "quaternion multiply\n";
        quaternion a(11 / 2.0, -2, -3, -2);
        quaternion b(11, -2, 0, -2);
        quaternion c = Mult(a, b);
        WeakEqual(c.w, 105 / 2.0, precision);
        WeakEqual(c.x, -27.0, precision);
        WeakEqual(c.y, -33.0, precision);
        WeakEqual(c.z, -39.0, precision);
    }

    // =============================================================================
    {
        std::cout << "quaternion multiply\n";
        quaternion R1 = Normalize(quaternion(rand(), rand(), rand(), rand()));
        quaternion R2 = Normalize(quaternion(rand(), rand(), rand(), rand()));

        quaternion Res1 = Mult(R1, R2);
        ChQuaternion<real> Res2;
        Res2.Cross(ToChQuaternion(R1), ToChQuaternion(R2));
        WeakEqual(Res1, ToQuaternion(Res2), precision);
    }

    // =============================================================================
    {
        std::cout << "quaternion rotate\n";
        real3 a(1.0, 2.0, -3.0);
        quaternion b(2.0, -2, -2, -2);
        b = Normalize(b);

        real3 c = Rotate(a, b);
        WeakEqual(c.x, 2, precision);
        WeakEqual(c.y, -3, precision);
        WeakEqual(c.z, 1, precision);
    }
    // =============================================================================

    // =============================================================================
    {
        std::cout << "quaternion rotate\n";
        real3 a(1.0, 2.0, -3.0);
        quaternion b(2.0, -2, -2, -2);
        b = Normalize(b);
        real3 c = RotateT(a, b);
        WeakEqual(c.x, -3, precision);
        WeakEqual(c.y, 1, precision);
        WeakEqual(c.z, 2, precision);
    }

    // =============================================================================
    {
        std::cout << "real4 rotate\n";
        real3 V(1.0, 2.0, -3.0);
        quaternion R1 = Normalize(quaternion(rand(), rand(), rand(), rand()));
        real3 Res1a = Rotate(V, R1);
        ChQuaternion<real> R2 = ToChQuaternion(R1);

        ChVector<real> Res2 = R2.Rotate(ToChVector(V));
        WeakEqual(Res1a, ToReal3(Res2), precision);
    }
    // =============================================================================
    {
        std::cout << "quaternion conjugate\n";

        quaternion R1 = Normalize(quaternion(rand(), rand(), rand(), rand()));
        quaternion Res1 = ~R1;
        ChQuaternion<real> R2 = ToChQuaternion(R1);
        R2.Conjugate();
        ChQuaternion<real> Res2 = R2;
        WeakEqual(Res1, ToQuaternion(Res2), precision);
    }

    {
        std::cout << "quaternion AbsRotate\n";
        quaternion R1 = Normalize(quaternion(rand(), rand(), rand(), rand()));
        real3 V(1.0, 2.0, 3.0);
        real3 result_1 = AbsRotate(R1, V);

        Mat33 R2(R1);
        R2 = Abs(R2);
        real3 result_2 = R2 * V;

        WeakEqual(result_1, result_2, precision * 2);
    }

    // =============================================================================

    return 0;
}
