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
// ChronoParallel unit testing common functions
// =============================================================================

#include <stdio.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <float.h>
#include <chrono_parallel/math/ChParallelMath.h>
float COMPARE_EPS = FLT_EPSILON*5;

void StrictEqual(
      const float & x,
      const float & y) {
   if (x != y) {
      std::cout << x << " does not equal " << y << std::endl;
      exit(1);
   }
}

void StrictEqual(
      const real3 & a,
      const real3 & b) {
   StrictEqual(a.x, b.x);
   StrictEqual(a.y, b.y);
   StrictEqual(a.z, b.z);
}

void WeakEqual(
      const float & x,
      const float & y) {
   if (fabs(x - y) > COMPARE_EPS) {
      std::cout << x << " does not equal " << y << " " << fabs(x - y) << std::endl;
      exit(1);
   }
}

void WeakEqual(
      const real3 & a,
      const real3 & b) {
   WeakEqual(a.x, b.x);
   WeakEqual(a.y, b.y);
   WeakEqual(a.z, b.z);
}
