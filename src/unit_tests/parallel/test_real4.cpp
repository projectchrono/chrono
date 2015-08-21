// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#include <stdio.h>
#include <vector>
#include <cmath>

#include "unit_testing.h"

#include "chrono_parallel/math/real4.h"

#ifdef CHRONO_PARALLEL_USE_DOUBLE
const double precision = 1e-10;
#else
const float precision = 1e-6f;
#endif

int main(int argc, char* argv[]) {
  // =============================================================================
  {
    std::cout << "real4 inverse\n";
    real4 a(11, -2, 0, -2);
    real4 b = inv(a);
    StrictEqual(b.w, 11.0 / 129.0);
    StrictEqual(b.x, 2.0 / 129.0);
    StrictEqual(b.y, 0.0);
    StrictEqual(b.z, 2.0 / 129.0);
  }
  // =============================================================================
  {
    std::cout << "real4 normalize\n";
    real4 a(11, -2, 0, -2);
    real4 b = normalize(a);
    WeakEqual(b.w, 11.0 / sqrt(129.0), precision);
    WeakEqual(b.x, -2.0 / sqrt(129.0), precision);
    WeakEqual(b.y, 0.0, precision);
    WeakEqual(b.z, -2.0 / sqrt(129.0), precision);
  }
  // =============================================================================
  {
    std::cout << "real4 multiply\n";
    real4 a(11 / 2.0, -2, -3, -2);
    real4 b(11, -2, 0, -2);
    real4 c = mult(a, b);
    WeakEqual(c.w, 105 / 2.0, precision);
    WeakEqual(c.x, -27.0, precision);
    WeakEqual(c.y, -33.0, precision);
    WeakEqual(c.z, -39.0, precision);
  }

  // =============================================================================
  {
    std::cout << "real4 multiply\n";
    real4 R1 = normalize(real4(rand(), rand(), rand(), rand()));
    real4 R2 = normalize(real4(rand(), rand(), rand(), rand()));

    real4 Res1 = mult(R1, R2);
    ChQuaternion<real> Res2;
    Res2.Cross(ToChQuaternion(R1), ToChQuaternion(R2));
    WeakEqual(Res1, ToReal4(Res2), precision);
  }

  // =============================================================================
  {
    std::cout << "real4 rotate\n";
    real3 a(1.0, 2.0, -3.0);
    real4 b(2.0, -2, -2, -2);
    b = normalize(b);

    real3 c = quatRotate(a, b);
    WeakEqual(c.x, 2, precision);
    WeakEqual(c.y, -3, precision);
    WeakEqual(c.z, 1, precision);
  }
  // =============================================================================
  {
    std::cout << "real4 rotate\n";
    real4 b(11 / 2.0, -2, -3, -2);
    real4 c = ~b;
    WeakEqual(c.w, 5.5, precision);
    WeakEqual(c.x, 2.0, precision);
    WeakEqual(c.y, 3.0, precision);
    WeakEqual(c.z, 2.0, precision);
  }

  // =============================================================================
  {
    std::cout << "real4 rotate\n";
    real3 a(1.0, 2.0, -3.0);
    real4 b(2.0, -2, -2, -2);
    b = normalize(b);
    real3 c = quatRotateMatT(a, b);
    WeakEqual(c.x, -3, precision);
    WeakEqual(c.y, 1, precision);
    WeakEqual(c.z, 2, precision);
  }

  // =============================================================================
  {
    std::cout << "real4 rotate\n";
    real3 V(1.0, 2.0, -3.0);
    real4 R1 = normalize(real4(rand(), rand(), rand(), rand()));
    real3 Res1a = quatRotate(V, R1);
    real3 Res1b = quatRotateMat(V, R1);
    ChQuaternion<real> R2 = ToChQuaternion(R1);

    ChVector<real> Res2 = R2.Rotate(ToChVector(V));
    WeakEqual(Res1a, ToReal3(Res2), precision);
    WeakEqual(Res1b, ToReal3(Res2), precision);
  }
  // =============================================================================
  {
    std::cout << "real4 conjugate\n";

    real4 R1 = normalize(real4(rand(), rand(), rand(), rand()));
    real4 Res1 = ~R1;
    ChQuaternion<real> R2 = ToChQuaternion(R1);
    R2.Conjugate();
    ChQuaternion<real> Res2 = R2;
    WeakEqual(Res1, ToReal4(Res2), precision);
  }
  // =============================================================================

  return 0;
}
