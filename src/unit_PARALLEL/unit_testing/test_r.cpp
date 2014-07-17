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

// =============================================================================
// Tests for various utility functions
// =============================================================================

void test_snap_to_box()
{
  cout << "snap_to_box" << endl;
  real3 hdims(1.0, 2.0, 3.0);

  {
    cout << "  interior point" << endl;
    real3 loc(0.5, -1.0, 1.5);
    int code = snap_to_box(hdims, loc);
    StrictEqual(code, 0);
    StrictEqual(loc, real3(0.5, -1.0, 1.5));
  }

  {
    cout << "  face point" << endl;
    real3 loc(0.5, -1.0, -3.5);
    int code = snap_to_box(hdims, loc);
    StrictEqual(code, 4);
    StrictEqual(loc, real3(0.5, -1.0, -3.0));
  }

  {
    cout << "  edge point" << endl;
    real3 loc(0.5, -2.5, -3.5);
    int code = snap_to_box(hdims, loc);
    StrictEqual(code, 6);
    StrictEqual(loc, real3(0.5, -2.0, -3.0));
  }

  {
    cout << "  corner point" << endl;
    real3 loc(1.5, -2.5, -3.5);
    int code = snap_to_box(hdims, loc);
    StrictEqual(code, 7);
    StrictEqual(loc, real3(1.0, -2.0, -3.0));
  }

}

void test_snap_to_cylinder()
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

// =============================================================================
// Tests for various primitive collision functions
// =============================================================================

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

void test_box_sphere()
{
  cout << "box_sphere" << endl;

  real3 b_hdims(1.0, 2.0, 3.0);
  real  s_rad = 1.5;

  // Box position and orientation fixed for all tests.
  // Rotated by 45 degrees around Z axis and shifted by sqrt(2) in X direction.
  real3 b_pos(sqrt(2.0), 0.0, 0.0);
  real4 b_rot = ToReal4(Q_from_AngAxis(CH_C_PI_4, ChVector<>(0, 0, 1)));

  real3 norm;
  real  depth;
  real3 pt1;
  real3 pt2;
  real  eff_rad;

  float oosqrt2 = sqrt(0.5);   // 1/sqrt(2)

  {
    cout << "  sphere center inside box" << endl;
    real3 s_pos(0.5, 0.5, 1.0);
    bool res = ChCNarrowphaseR::box_sphere(b_pos, b_rot, b_hdims,
                                           s_pos, s_rad,
                                           norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  face interaction (separated)" << endl;
    real3 s_pos(3.0, 2.0, 1.0);
    bool res = ChCNarrowphaseR::box_sphere(b_pos, b_rot, b_hdims,
                                           s_pos, s_rad,
                                           norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  face interaction (separated)" << endl;
    real3 s_pos(4 * oosqrt2, 2.0 * oosqrt2, 1.0);
    bool res = ChCNarrowphaseR::box_sphere(b_pos, b_rot, b_hdims,
                                           s_pos, s_rad,
                                           norm, depth, pt1, pt2, eff_rad);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    WeakEqual(norm, real3(oosqrt2, oosqrt2, 0.0));
    WeakEqual(depth, -0.5);
    WeakEqual(pt1, real3(3.0 * oosqrt2, oosqrt2, 1.0));
    WeakEqual(pt2, real3(2.5 * oosqrt2, 0.5 * oosqrt2, 1.0));
    WeakEqual(eff_rad, s_rad);
  }

  {
    cout << "  edge interaction (separated)" << endl;
    real3 s_pos(oosqrt2, 4.0, 1.0);
    bool res = ChCNarrowphaseR::box_sphere(b_pos, b_rot, b_hdims,
                                           s_pos, s_rad,
                                           norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  edge interaction (penetrated)" << endl;
    real3 s_pos(oosqrt2, 3.0 * oosqrt2 + 1.0, 1.0);
    bool res = ChCNarrowphaseR::box_sphere(b_pos, b_rot, b_hdims,
                                           s_pos, s_rad,
                                           norm, depth, pt1, pt2, eff_rad);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    WeakEqual(norm, real3(0.0, 1.0, 0.0));
    WeakEqual(depth, -0.5);
    WeakEqual(pt1, real3(oosqrt2, 3.0 * oosqrt2, 1.0));
    WeakEqual(pt2, real3(oosqrt2, 3.0 * oosqrt2 - 0.5, 1.0));
    WeakEqual(eff_rad, s_rad * ChCNarrowphaseR::edge_radius/(s_rad + ChCNarrowphaseR::edge_radius));
  }

  {
    cout << "  corner interaction (separated)" << endl;
    real3 s_pos(oosqrt2, 4.0, 4.0);
    bool res = ChCNarrowphaseR::box_sphere(b_pos, b_rot, b_hdims,
                                           s_pos, s_rad,
                                           norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  corner interaction (penetrated)" << endl;
    real3 s_pos(oosqrt2, 4.0 * oosqrt2, 3.0 + oosqrt2);
    bool res = ChCNarrowphaseR::box_sphere(b_pos, b_rot, b_hdims,
                                           s_pos, s_rad,
                                           norm, depth, pt1, pt2, eff_rad);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    WeakEqual(norm, real3(0.0, oosqrt2, oosqrt2));
    WeakEqual(depth, -0.5);
    WeakEqual(pt1, real3(oosqrt2, 3.0 * oosqrt2, 3.0));
    WeakEqual(pt2, s_pos - s_rad * norm);
    WeakEqual(eff_rad, s_rad * ChCNarrowphaseR::edge_radius/(s_rad + ChCNarrowphaseR::edge_radius));
  }
}

// -----------------------------------------------------------------------------

void test_capsule_sphere()
{
  cout << "capsule_sphere" << endl;

  real c_rad = 0.5;
  real c_hlen = 2.0;

  real s_rad = 1.0;

  // Capsule position and orientation fixed for all tests.
  // aligned with X axis and shifted by its half-length in the X direction.
  real3 c_pos(c_hlen, 0, 0);
  real4 c_rot = ToReal4(Q_from_AngAxis(CH_C_PI_2, ChVector<>(0, 0, 1)));

  real3 norm;
  real  depth;
  real3 pt1;
  real3 pt2;
  real  eff_rad;

  float oosqrt2 = sqrt(0.5);   // 1/sqrt(2)

  {
    cout << "  sphere center on capsule axis" << endl;
    real3 s_pos(3.0, 0.0, 0.0);
    bool res = ChCNarrowphaseR::capsule_sphere(c_pos, c_rot, c_rad, c_hlen,
                                               s_pos, s_rad,
                                               norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  cap interaction (separated)" << endl;
    real3 s_pos(5.0, 1.5, 0.0);
    bool res = ChCNarrowphaseR::capsule_sphere(c_pos, c_rot, c_rad, c_hlen,
                                               s_pos, s_rad,
                                               norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  cap interaction (penetrated)" << endl;
    real3 s_pos(5.0, 1.0, 0.0);
    bool res = ChCNarrowphaseR::capsule_sphere(c_pos, c_rot, c_rad, c_hlen,
                                               s_pos, s_rad,
                                               norm, depth, pt1, pt2, eff_rad);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    WeakEqual(norm, real3(oosqrt2, oosqrt2, 0));
    WeakEqual(depth, sqrt(2.0) - 1.5);
    WeakEqual(pt1, real3(4.0 + 0.5*oosqrt2, 0.5*oosqrt2, 0));
    WeakEqual(pt2, real3(5.0 - oosqrt2, 1.0 - oosqrt2, 0));
    WeakEqual(eff_rad, s_rad * c_rad/(s_rad + c_rad));
  }

  {
    cout << "  side interaction (separated)" << endl;
    real3 s_pos(2.5, 2.0, 0);
    bool res = ChCNarrowphaseR::capsule_sphere(c_pos, c_rot, c_rad, c_hlen,
                                               s_pos, s_rad,
                                               norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  side interaction (penetrated)" << endl;
    real3 s_pos(2.5, 1.25, 0);
    bool res = ChCNarrowphaseR::capsule_sphere(c_pos, c_rot, c_rad, c_hlen,
                                               s_pos, s_rad,
                                               norm, depth, pt1, pt2, eff_rad);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    WeakEqual(norm, real3(0, 1, 0));
    WeakEqual(depth, -0.25);
    WeakEqual(pt1, real3(2.5, 0.5, 0));
    WeakEqual(pt2, real3(2.5, 0.25, 0));
    WeakEqual(eff_rad, s_rad * c_rad/(s_rad + c_rad));
  }
}

// -----------------------------------------------------------------------------

void test_cylinder_sphere()
{
  cout << "cylinder_sphere" << endl;

  real  c_rad = 2.0;
  real  c_hlen = 1.5;
  real  s_rad = 1.0;

  real3 c_pos(c_hlen, 0, 0);
  real4 c_rot = ToReal4(Q_from_AngAxis(CH_C_PI_2, ChVector<>(0, 0, 1)));

  real3 norm;
  real  depth;
  real3 pt1;
  real3 pt2;
  real  eff_rad;

  float oosqrt2 = sqrt(0.5);   // 1/sqrt(2)

  {
    cout << "  sphere center inside cylinder" << endl;
    real3 s_pos(2.5, 1.5, 0);
    bool res = ChCNarrowphaseR::cylinder_sphere(c_pos, c_rot, c_rad, c_hlen,
                                                s_pos, s_rad,
                                                norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  cap interaction (separated)" << endl;
    real3 s_pos(4.5, 1.5, 0);
    bool res = ChCNarrowphaseR::cylinder_sphere(c_pos, c_rot, c_rad, c_hlen,
                                                s_pos, s_rad,
                                                norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  cap interaction (penetrated)" << endl;
    real3 s_pos(3.75, 1.5, 0);
    bool res = ChCNarrowphaseR::cylinder_sphere(c_pos, c_rot, c_rad, c_hlen,
                                                s_pos, s_rad,
                                                norm, depth, pt1, pt2, eff_rad);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    WeakEqual(norm, real3(1, 0, 0));
    WeakEqual(depth, -0.25);
    WeakEqual(pt1, real3(3, 1.5, 0));
    WeakEqual(pt2, real3(2.75, 1.5, 0));
    WeakEqual(eff_rad, s_rad);
  }

  {
    cout << "  side interaction (separated)" << endl;
    real3 s_pos(2.5, 3.5, 0);
    bool res = ChCNarrowphaseR::cylinder_sphere(c_pos, c_rot, c_rad, c_hlen,
                                                s_pos, s_rad,
                                                norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  side interaction (penetrated)" << endl;
    real3 s_pos(2.5, 2.5, 0);
    bool res = ChCNarrowphaseR::cylinder_sphere(c_pos, c_rot, c_rad, c_hlen,
                                                s_pos, s_rad,
                                                norm, depth, pt1, pt2, eff_rad);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    WeakEqual(norm, real3(0, 1, 0));
    WeakEqual(depth, -0.5);
    WeakEqual(pt1, real3(2.5, 2.0, 0));
    WeakEqual(pt2, real3(2.5, 1.5, 0));
    WeakEqual(eff_rad, s_rad * c_rad/(s_rad + c_rad));
  }

  {
    cout << "  edge interaction (separated)" << endl;
    real3 s_pos(4, 3, 0);
    bool res = ChCNarrowphaseR::cylinder_sphere(c_pos, c_rot, c_rad, c_hlen,
                                                s_pos, s_rad,
                                                norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  edge interaction (penetrated)" << endl;
    real3 s_pos(3.5, 2.5, 0);
    bool res = ChCNarrowphaseR::cylinder_sphere(c_pos, c_rot, c_rad, c_hlen,
                                                s_pos, s_rad,
                                                norm, depth, pt1, pt2, eff_rad);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    WeakEqual(norm, real3(oosqrt2, oosqrt2, 0));
    WeakEqual(depth, -1 + oosqrt2);
    WeakEqual(pt1, real3(3.0, 2.0, 0));
    WeakEqual(pt2, real3(3.5 - oosqrt2, 2.5 - oosqrt2, 0));
    WeakEqual(eff_rad, s_rad * ChCNarrowphaseR::edge_radius/(s_rad + ChCNarrowphaseR::edge_radius));
  }
}

// -----------------------------------------------------------------------------

void test_roundedcyl_sphere()
{
  cout << "roundedcyl_sphere" << endl;

  real  c_rad = 2.0;   // radius of skeleton cylinder
  real  c_hlen = 1.5;  // half-length of skeleton cylinder
  real  c_srad = 0.1;  // radius of sweeping sphere

  real  s_rad = 1.0;   // sphere radius

  // Rounded cylinder position and orientation fixed for all tests.
  // Aligned with X axis and shifted by its half-length in the X direction.
  real3 c_pos(c_hlen, 0, 0);
  real4 c_rot = ToReal4(Q_from_AngAxis(CH_C_PI_2, ChVector<>(0, 0, 1)));

  real3 norm;
  real  depth;
  real3 pt1;
  real3 pt2;
  real  eff_rad;

  float oosqrt2 = sqrt(0.5);   // 1/sqrt(2)

  {
    cout << "  sphere center inside cylinder" << endl;
    real3 s_pos(2.5, 1.5, 0);
    bool res = ChCNarrowphaseR::roundedcyl_sphere(c_pos, c_rot, c_rad, c_hlen, c_srad,
                                                  s_pos, s_rad,
                                                  norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  cap interaction (separated)" << endl;
    real3 s_pos(4.5, 1.5, 0);
    bool res = ChCNarrowphaseR::roundedcyl_sphere(c_pos, c_rot, c_rad, c_hlen, c_srad,
                                                  s_pos, s_rad,
                                                  norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  cap interaction (penetrated)" << endl;
    real3 s_pos(3.75, 1.5, 0);
    bool res = ChCNarrowphaseR::roundedcyl_sphere(c_pos, c_rot, c_rad, c_hlen, c_srad,
                                                  s_pos, s_rad,
                                                  norm, depth, pt1, pt2, eff_rad);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    WeakEqual(norm, real3(1, 0, 0));
    WeakEqual(depth, -0.35);
    WeakEqual(pt1, real3(3.1, 1.5, 0));
    WeakEqual(pt2, real3(2.75, 1.5, 0));
    WeakEqual(eff_rad, s_rad);
  }

  {
    cout << "  side interaction (separated)" << endl;
    real3 s_pos(2.5, 3.5, 0);
    bool res = ChCNarrowphaseR::roundedcyl_sphere(c_pos, c_rot, c_rad, c_hlen, c_srad,
                                                  s_pos, s_rad,
                                                  norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  side interaction (penetrated)" << endl;
    real3 s_pos(2.5, 2.5, 0);
    bool res = ChCNarrowphaseR::roundedcyl_sphere(c_pos, c_rot, c_rad, c_hlen, c_srad,
                                                  s_pos, s_rad,
                                                  norm, depth, pt1, pt2, eff_rad);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    WeakEqual(norm, real3(0, 1, 0));
    WeakEqual(depth, -0.6);
    WeakEqual(pt1, real3(2.5, 2.1, 0));
    WeakEqual(pt2, real3(2.5, 1.5, 0));
    WeakEqual(eff_rad, s_rad * c_rad/(s_rad + c_rad));
  }

  {
    cout << "  edge interaction (separated)" << endl;
    real3 s_pos(4, 3, 0);
    bool res = ChCNarrowphaseR::roundedcyl_sphere(c_pos, c_rot, c_rad, c_hlen, c_srad,
                                                  s_pos, s_rad,
                                                  norm, depth, pt1, pt2, eff_rad);
    if (res) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  edge interaction (penetrated)" << endl;
    real3 s_pos(3.5, 2.5, 0);
    bool res = ChCNarrowphaseR::roundedcyl_sphere(c_pos, c_rot, c_rad, c_hlen, c_srad,
                                                  s_pos, s_rad,
                                                  norm, depth, pt1, pt2, eff_rad);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    WeakEqual(norm, real3(oosqrt2, oosqrt2, 0));
    WeakEqual(depth, -1.1 + oosqrt2);
    WeakEqual(pt1, real3(3.0 + 0.1*oosqrt2, 2.0 + 0.1*oosqrt2, 0));
    WeakEqual(pt2, real3(3.5 - oosqrt2, 2.5 - oosqrt2, 0));
    WeakEqual(eff_rad, s_rad * c_srad/(s_rad + c_srad));
  }
}

// =============================================================================

int main()
{
  // Utility functions
  test_snap_to_box();
  test_snap_to_cylinder();

  // Collision detection
  test_sphere_sphere();
  test_box_sphere();
  test_capsule_sphere();
  test_cylinder_sphere();
  test_roundedcyl_sphere();

   return 0;
}

