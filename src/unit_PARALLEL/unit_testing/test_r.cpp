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
// Authors: Radu Serban
// =============================================================================
//
// ChronoParallel unit test for narrow phase type R collision detection
// =============================================================================

#include <stdio.h>
#include <vector>
#include <cmath>

#include "collision/ChCCollisionModel.h"
#include "core/ChMathematics.h"

#include "chrono_parallel/collision/ChCNarrowphaseR.h"
#include "chrono_parallel/collision/ChCNarrowphaseRUtils.h"
#include "unit_testing.h"

using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::endl;

// -----------------------------------------------------------------------------
// Tests for various utility functions
// -----------------------------------------------------------------------------
void
test_snap_to_cylinder()
{
  cout << "snap_to_cylinder" << endl;
  real rad = 2;
  real hlen = 1.5;

  {
    cout << "  interior point" << endl;
    real3 loc(0.5, -1.0, 1.5);
    int code = snap_to_cylinder(rad, hlen, loc);
    StrictEqual(code, 0);
    StrictEqual(loc, real3(0.5, -1.0, 1.5));
  }

  {
    cout << "  cap point" << endl;
    real3 loc(0.5, 2.0, 1.5);
    int code = snap_to_cylinder(rad, hlen, loc);
    StrictEqual(code, 1);
    StrictEqual(loc, real3(0.5, 1.5, 1.5));
  }

  {
    cout << "  side point" << endl;
    real3 loc(2.0, 0.5, 1.0);
    int code = snap_to_cylinder(rad, hlen, loc);
    StrictEqual(code, 2);
    WeakEqual(loc, real3(4/sqrt(5.0), 0.5, 2/sqrt(5.0)));
  }

  {
    cout << "  edge point" << endl;
    real3 loc(2.0, 2.0, 1.0);
    int code = snap_to_cylinder(rad, hlen, loc);
    StrictEqual(code, 3);
    WeakEqual(loc, real3(4/sqrt(5.0), 1.5, 2/sqrt(5.0)));
  }
}

// -----------------------------------------------------------------------------
// Tests for various primitive collision functions
// -----------------------------------------------------------------------------
void test_sphere_sphere()
{
  cout << "sphere_sphere" << endl;
  real3 norm;
  real  depth;
  real3 pt1;
  real3 pt2;
  real  eff_rad;

    {
    cout << "  separated" << endl;
    bool res = ChCNarrowphaseR::sphere_sphere(real3(2, 2, 0), 1.0,
                                              real3(2, 0, 0), 0.5,
                                              norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  touching" << endl;
    bool res = ChCNarrowphaseR::sphere_sphere(real3(2, 2, 0), 1.0,
                                              real3(2, 0, 0), 1.0,
                                              norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  penetrated" << endl;
    bool res = ChCNarrowphaseR::sphere_sphere(real3(1, 1, 0), 1.0,
                                              real3(2.5, 1, 0), 1.0,
                                              norm, depth, pt1, pt2, eff_rad);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    WeakEqual(norm, real3(1, 0, 0));
    WeakEqual(depth, -0.5);
    WeakEqual(pt1, real3(2, 1, 0));
    WeakEqual(pt2, real3(1.5, 1, 0));
    WeakEqual(eff_rad, 0.5);
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
int main()
{
  // Utility functions
  test_snap_to_cylinder();

  // Collision detection
  test_sphere_sphere();

   return 0;
}

