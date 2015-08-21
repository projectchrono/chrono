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

#include "chrono_parallel/math/real3.h"

#ifdef CHRONO_PARALLEL_USE_DOUBLE
const double precision = 1e-10;
#else
const float precision = 1e-6f;
#endif

int main(int argc, char* argv[]) {
  // =============================================================================
  {  // zero constructor
    real3 zero;
    WeakEqual(zero.x, 0.0, precision);
    WeakEqual(zero.y, 0.0, precision);
    WeakEqual(zero.z, 0.0, precision);
  }

  {  // 1 float constructor
    real3 value(1.5);
    WeakEqual(value.x, 1.5, precision);
    WeakEqual(value.y, 1.5, precision);
    WeakEqual(value.z, 1.5, precision);
  }

  {  // 3 float constructor
    real3 value(1.5, 2.6, 3.7);
    WeakEqual(value.x, 1.5, precision);
    WeakEqual(value.y, 2.6, precision);
    WeakEqual(value.z, 3.7, precision);
  }
  // =============================================================================
  {  // float 3 add
    real3 a(1.0, 2.0, 3.0);
    real3 b(3.0, 2.0, 1.0);
    real3 c = a + b;
    WeakEqual(c.x, 4.0, precision);
    WeakEqual(c.y, 4.0, precision);
    WeakEqual(c.z, 4.0, precision);
  }

  {  // float 3 subtract
    real3 a(1.0, 2.0, 3.0);
    real3 b(3.0, 2.0, 1.0);
    real3 c = a - b;
    WeakEqual(c.x, -2.0, precision);
    WeakEqual(c.y, 0.0, precision);
    WeakEqual(c.z, 2.0, precision);
  }

  {  // float 3 multiply
    real3 a(1.0, 2.0, 3.0);
    real3 b(3.0, 2.0, 1.0);
    real3 c = a * b;
    WeakEqual(c.x, 3.0, precision);
    WeakEqual(c.y, 4.0, precision);
    WeakEqual(c.z, 3.0, precision);
  }

  {  // float 3 divide
    real3 a(1.0, 2.0, 3.0);
    real3 b(3.0, 2.0, 1.0);
    real3 c = a / b;
    WeakEqual(c.x, 1.0 / 3.0, precision);
    WeakEqual(c.y, 2.0 / 2.0, precision);
    WeakEqual(c.z, 3.0 / 1.0, precision);
  }
  {  // float 3 negate
    real3 a(1.0, 2.0, 3.0);
    real3 c = -a;
    WeakEqual(c.x, -1.0, precision);
    WeakEqual(c.y, -2.0, precision);
    WeakEqual(c.z, -3.0, precision);
  }
  // =============================================================================
  {  // float 3 add
    real3 a(1.0, 2.0, 3.0);
    real3 b(3.0, 2.0, 1.0);
    a += b;
    WeakEqual(a.x, 4.0, precision);
    WeakEqual(a.y, 4.0, precision);
    WeakEqual(a.z, 4.0, precision);
  }

  {  // float 3 subtract
    real3 a(1.0, 2.0, 3.0);
    real3 b(3.0, 2.0, 1.0);
    a -= b;
    WeakEqual(a.x, -2.0, precision);
    WeakEqual(a.y, 0.0, precision);
    WeakEqual(a.z, 2.0, precision);
  }

  {  // float 3 multiply
    real3 a(1.0, 2.0, 3.0);
    real3 b(3.0, 2.0, 1.0);
    a *= b;
    WeakEqual(a.x, 3.0, precision);
    WeakEqual(a.y, 4.0, precision);
    WeakEqual(a.z, 3.0, precision);
  }

  {  // float 3 divide
    real3 a(1.0, 2.0, 3.0);
    real3 b(3.0, 2.0, 1.0);
    a /= b;
    WeakEqual(a.x, 1.0 / 3.0, precision);
    WeakEqual(a.y, 2.0 / 2.0, precision);
    WeakEqual(a.z, 3.0 / 1.0, precision);
  }
  // =============================================================================

  {  // float 3 add
    real3 a(1.0, 2.0, 3.0);
    real b(2.0);
    real3 c = a + b;
    WeakEqual(c.x, 3.0, precision);
    WeakEqual(c.y, 4.0, precision);
    WeakEqual(c.z, 5.0, precision);
  }

  {  // float 3 subtract
    real3 a(1.0, 2.0, 3.0);
    real b(2.0);
    real3 c = a - b;
    WeakEqual(c.x, -1.0, precision);
    WeakEqual(c.y, 0.0, precision);
    WeakEqual(c.z, 1.0, precision);
  }

  {  // float 3 multiply
    real3 a(1.0, 2.0, 3.0);
    real b(2.0);
    real3 c = a * b;
    WeakEqual(c.x, 2.0, precision);
    WeakEqual(c.y, 4.0, precision);
    WeakEqual(c.z, 6.0, precision);
  }

  {  // float 3 divide
    real3 a(1.0, 2.0, 3.0);
    real b(2.0);
    real3 c = a / b;
    WeakEqual(c.x, 1.0 / 2.0, precision);
    WeakEqual(c.y, 2.0 / 2.0, precision);
    WeakEqual(c.z, 3.0 / 2.0, precision);
  }
  // =============================================================================

  {  // float 3 add
    real3 a(1.0, 2.0, 3.0);
    real b(2.0);
    a += b;
    WeakEqual(a.x, 3.0, precision);
    WeakEqual(a.y, 4.0, precision);
    WeakEqual(a.z, 5.0, precision);
  }

  {  // float 3 subtract
    real3 a(1.0, 2.0, 3.0);
    real b(2.0);
    a -= b;
    WeakEqual(a.x, -1.0, precision);
    WeakEqual(a.y, 0.0, precision);
    WeakEqual(a.z, 1.0, precision);
  }

  {  // float 3 multiply
    real3 a(1.0, 2.0, 3.0);
    real b(2.0);
    a *= b;
    WeakEqual(a.x, 2.0, precision);
    WeakEqual(a.y, 4.0, precision);
    WeakEqual(a.z, 6.0, precision);
  }

  {  // float 3 divide
    real3 a(1.0, 2.0, 3.0);
    real b(2.0);
    a /= b;
    WeakEqual(a.x, 1.0 / 2.0, precision);
    WeakEqual(a.y, 2.0 / 2.0, precision);
    WeakEqual(a.z, 3.0 / 2.0, precision);
  }
  // =============================================================================

  {  // float 3 dot
    real3 a(1.0, 2.0, 3.0);
    real3 b(2.0, 1.0, 3.0);
    real c = dot(a, b);
    WeakEqual(c, 13.0, precision);
  }
  {  // float 3 cross
    real3 a(1.0, 2.0, 3.0);
    real3 b(2.0, 1.0, 3.0);
    real3 c = cross(a, b);
    WeakEqual(c.x, 3.0, precision);
    WeakEqual(c.y, 3.0, precision);
    WeakEqual(c.z, -3.0, precision);
  }
  {  // float 3 cross
    real3 a = normalize(real3(rand(), rand(), rand()));
    real3 b = normalize(real3(rand(), rand(), rand()));
    real3 ans1 = cross(a, b);
    ChVector<real> ans2;
    ans2.Cross(ToChVector(a), ToChVector(b));
    WeakEqual(ans1, ToReal3(ans2), precision);
  }

  {  // float 3 length
    real3 a(1.0, 2.0, -3.0);
    real c = length(a);
    WeakEqual(c, sqrt(14.0), precision);
  }
  {  // float 3 normalize
    real3 a(1.0, 2.0, -3.0);
    real3 c = normalize(a);
    WeakEqual(c.x, 1.0 / sqrt(14.0), precision);
    WeakEqual(c.y, 2.0 / sqrt(14.0), precision);
    WeakEqual(c.z, -3.0 / sqrt(14.0), precision);
  }

  return 0;
}
