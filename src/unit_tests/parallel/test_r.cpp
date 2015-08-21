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

#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/core/ChMathematics.h"

#include "chrono_parallel/collision/ChCNarrowphaseR.h"
#include "chrono_parallel/collision/ChCNarrowphaseRUtils.h"

#include "unit_testing.h"

using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::endl;

#ifdef CHRONO_PARALLEL_USE_DOUBLE
const double precision = 1e-10;
#else
const float precision = 1e-6f;
#endif

// =============================================================================
// Tests for various utility functions
// =============================================================================

void test_snap_to_box() {
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

void test_snap_to_cylinder() {
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
    WeakEqual(loc, real3(4 / sqrt(5.0), 0.5, 2 / sqrt(5.0)), precision);
  }

  {
    cout << "  edge point" << endl;
    real3 loc(2.0, 2.0, 1.0);
    int code = snap_to_cylinder(rad, hlen, loc);
    StrictEqual(code, 3);
    WeakEqual(loc, real3(4 / sqrt(5.0), 1.5, 2 / sqrt(5.0)), precision);
  }
}

// =============================================================================
// Tests for various primitive collision functions
// =============================================================================

void test_sphere_sphere(bool sep) {
  cout << "sphere_sphere" << endl;

  ConvexShape shapeS1;
  shapeS1.type = ShapeType::SPHERE;
  shapeS1.C = real3(0);
  shapeS1.R = real4(1, 0, 0, 0);

  ConvexShape shapeS2;
  shapeS2.type = ShapeType::SPHERE;
  shapeS2.C = real3(0);
  shapeS2.R = real4(1, 0, 0, 0);

  real separation = sep ? 0.1 : 0.0;

  // Output quantities.
  real3 norm;
  real3 pt1;
  real3 pt2;
  real depth;
  real eff_rad;
  int nC;

  {
    cout << "  separated (far)" << endl;

    shapeS1.A = real3(2, 2, 0);
    shapeS1.B = real3(1, 0, 0);

    shapeS2.A = real3(2, 0, 0);
    shapeS2.B = real3(0.5, 0, 0);

    bool res = RCollision(shapeS1, shapeS2, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res || nC != 0) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  separated (near)" << endl;

    shapeS1.A = real3(2, 2, 0);
    shapeS1.B = real3(1, 0, 0);

    shapeS2.A = real3(2, 0, 0);
    shapeS2.B = real3(0.95, 0, 0);

    bool res = RCollision(shapeS1, shapeS2, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (sep) {
      if (!res) {
        cout << "    test failed" << endl;
        exit(1);
      }
      StrictEqual(nC, 1);
      WeakEqual(norm, real3(0, -1, 0), precision);
      WeakEqual(depth, 0.05, precision);
      WeakEqual(pt1, real3(2, 1, 0), precision);
      WeakEqual(pt2, real3(2, 0.95, 0), precision);
      WeakEqual(eff_rad, 0.95 / 1.95, precision);
    } else {
      if (!res || nC != 0) {
        cout << "    test failed" << endl;
        exit(1);
      }
    }
  }

  {
    cout << "  touching" << endl;

    shapeS1.A = real3(2, 2, 0);
    shapeS1.B = real3(1, 0, 0);

    shapeS2.A = real3(2, 0, 0);
    shapeS2.B = real3(1, 0, 0);

    bool res = RCollision(shapeS1, shapeS2, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (sep) {
      if (!res) {
        cout << "    test failed" << endl;
        exit(1);
      }
      StrictEqual(nC, 1);
      WeakEqual(norm, real3(0, -1, 0), precision);
      WeakEqual(depth, 0, precision);
      WeakEqual(pt1, real3(2, 1, 0), precision);
      WeakEqual(pt2, real3(2, 1, 0), precision);
      WeakEqual(eff_rad, 0.5, precision);
    } else {
      if (!res || nC != 0) {
        cout << "    test failed" << endl;
        exit(1);
      }
    }
  }

  {
    cout << "  penetrated" << endl;

    shapeS1.A = real3(1, 1, 0);
    shapeS1.B = real3(1, 0, 0);

    shapeS2.A = real3(2.5, 1, 0);
    shapeS2.B = real3(1, 0, 0);

    bool res = RCollision(shapeS1, shapeS2, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    StrictEqual(nC, 1);
    WeakEqual(norm, real3(1, 0, 0), precision);
    WeakEqual(depth, -0.5, precision);
    WeakEqual(pt1, real3(2, 1, 0), precision);
    WeakEqual(pt2, real3(1.5, 1, 0), precision);
    WeakEqual(eff_rad, 0.5, precision);
  }
}

// -----------------------------------------------------------------------------

void test_box_sphere(bool sep) {
  cout << "box_sphere" << endl;

  // Box position and orientation fixed for all tests.
  // Rotated by 45 degrees around Z axis and shifted by sqrt(2) in X direction.
  real3 b_hdims(1.0, 2.0, 3.0);
  real3 b_pos(sqrt(2.0), 0.0, 0.0);
  real4 b_rot = ToReal4(Q_from_AngAxis(CH_C_PI_4, ChVector<>(0, 0, 1)));

  ConvexShape shapeB;
  shapeB.type = ShapeType::BOX;
  shapeB.A = b_pos;
  shapeB.B = b_hdims;
  shapeB.C = real3(0);
  shapeB.R = b_rot;

  // Sphere position changes for each test.
  real s_rad = 1.5;  // sphere radius

  ConvexShape shapeS;
  shapeS.type = ShapeType::SPHERE;
  shapeS.B = real3(s_rad, 0, 0);
  shapeS.C = real3(0);
  shapeS.R = real4(1, 0, 0, 0);

  real separation = sep ? 0.1 : 0.0;

  // Output quantities.
  real3 norm;
  real3 pt1;
  real3 pt2;
  real depth;
  real eff_rad;
  int nC;

  real oosqrt2 = sqrt(0.5);  // 1/sqrt(2)

  {
    cout << "  sphere center inside box" << endl;
    shapeS.A = real3(0.5, 0.5, 1.0);
    bool res = RCollision(shapeB, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res || nC != 0) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  face interaction (separated far)" << endl;
    shapeS.A = real3(3.5, 2.5, 1.0);
    bool res = RCollision(shapeB, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res || nC != 0) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  face interaction (separated near)" << endl;
    shapeS.A = real3(4.55 * oosqrt2, 2.55 * oosqrt2, 1.0);
    bool res = RCollision(shapeB, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (sep) {
      if (!res) {
        cout << "    test failed" << endl;
        exit(1);
      }
      StrictEqual(nC, 1);
      WeakEqual(norm, real3(oosqrt2, oosqrt2, 0.0), precision);
      WeakEqual(depth, 0.05, precision);
      WeakEqual(pt1, real3(3.0 * oosqrt2, oosqrt2, 1.0), precision);
      WeakEqual(pt2, real3(3.05 * oosqrt2, 1.05 * oosqrt2, 1.0), precision);
    } else {
      if (!res || nC != 0) {
        cout << "    test failed" << endl;
        exit(1);
      }
    }
  }

  {
    cout << "  face interaction (penetrated)" << endl;
    shapeS.A = real3(4 * oosqrt2, 2.0 * oosqrt2, 1.0);
    bool res = RCollision(shapeB, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    StrictEqual(nC, 1);
    WeakEqual(norm, real3(oosqrt2, oosqrt2, 0.0), precision);
    WeakEqual(depth, -0.5, precision);
    WeakEqual(pt1, real3(3.0 * oosqrt2, oosqrt2, 1.0), precision);
    WeakEqual(pt2, real3(2.5 * oosqrt2, 0.5 * oosqrt2, 1.0), precision);
    WeakEqual(eff_rad, s_rad, precision);
  }

  {
    cout << "  edge interaction (separated far)" << endl;
    shapeS.A = real3(oosqrt2, 4.0, 1.0);
    bool res = RCollision(shapeB, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res || nC != 0) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  edge interaction (separated near)" << endl;
    shapeS.A = real3(oosqrt2, 3.0 * oosqrt2 + 1.55, 1.0);
    bool res = RCollision(shapeB, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (sep) {
      if (!res) {
        cout << "    test failed" << endl;
        exit(1);
      }
      StrictEqual(nC, 1);
      WeakEqual(norm, real3(0.0, 1.0, 0.0), precision);
      WeakEqual(depth, 0.05, precision);
      WeakEqual(pt1, real3(oosqrt2, 3.0 * oosqrt2, 1.0), precision);
      WeakEqual(pt2, real3(oosqrt2, 3.0 * oosqrt2 + 0.05, 1.0), precision);
    } else {
      if (!res || nC != 0) {
        cout << "    test failed" << endl;
        exit(1);
      }
    }
  }

  {
    cout << "  edge interaction (penetrated)" << endl;
    shapeS.A = real3(oosqrt2, 3.0 * oosqrt2 + 1.0, 1.0);
    bool res = RCollision(shapeB, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    StrictEqual(nC, 1);
    WeakEqual(norm, real3(0.0, 1.0, 0.0), precision);
    WeakEqual(depth, -0.5, precision);
    WeakEqual(pt1, real3(oosqrt2, 3.0 * oosqrt2, 1.0), precision);
    WeakEqual(pt2, real3(oosqrt2, 3.0 * oosqrt2 - 0.5, 1.0), precision);
    WeakEqual(eff_rad, s_rad * edge_radius / (s_rad + edge_radius), precision);
  }

  {
    cout << "  corner interaction (separated far)" << endl;
    shapeS.A = real3(oosqrt2, 4.0, 4.0);
    bool res = RCollision(shapeB, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res || nC != 0) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  corner interaction (separated near)" << endl;
    real3 s_pos(oosqrt2, 4.55 * oosqrt2, 3.0 + 1.55 * oosqrt2);
    shapeS.A = s_pos;
    bool res = RCollision(shapeB, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (sep) {
      if (!res) {
        cout << "    test failed" << endl;
        exit(1);
      }
      StrictEqual(nC, 1);
      WeakEqual(norm, real3(0.0, oosqrt2, oosqrt2), precision);
      WeakEqual(depth, 0.05, precision);
      WeakEqual(pt1, real3(oosqrt2, 3.0 * oosqrt2, 3.0), precision);
      WeakEqual(pt2, s_pos - s_rad * norm, precision);
    } else {
      if (!res || nC != 0) {
        cout << "    test failed" << endl;
        exit(1);
      }
    }
  }

  {
    cout << "  corner interaction (penetrated)" << endl;
    real3 s_pos(oosqrt2, 4.0 * oosqrt2, 3.0 + oosqrt2);
    shapeS.A = s_pos;
    bool res = RCollision(shapeB, shapeS, separation, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    StrictEqual(nC, 1);
    WeakEqual(norm, real3(0.0, oosqrt2, oosqrt2), precision);
    WeakEqual(depth, -0.5, precision);
    WeakEqual(pt1, real3(oosqrt2, 3.0 * oosqrt2, 3.0), precision);
    WeakEqual(pt2, s_pos - s_rad * norm, precision);
    WeakEqual(eff_rad, s_rad * edge_radius / (s_rad + edge_radius), precision);
  }
}

// -----------------------------------------------------------------------------

void test_capsule_sphere() {
  cout << "capsule_sphere" << endl;

  // Capsule position and orientation fixed for all tests.
  // aligned with X axis and shifted by its half-length in the X direction.
  real c_rad = 0.5;
  real c_hlen = 2.0;
  real3 c_pos(c_hlen, 0, 0);
  real4 c_rot = ToReal4(Q_from_AngAxis(CH_C_PI_2, ChVector<>(0, 0, 1)));

  ConvexShape shapeC;
  shapeC.type = ShapeType::CAPSULE;
  shapeC.A = c_pos;
  shapeC.B = real3(c_rad, c_hlen, c_rad);
  shapeC.C = real3(0);
  shapeC.R = c_rot;

  // Sphere position changes for each test.
  real s_rad = 1.0;  // sphere radius

  ConvexShape shapeS;
  shapeS.type = ShapeType::SPHERE;
  shapeS.B = real3(s_rad, 0, 0);
  shapeS.C = real3(0);
  shapeS.R = real4(1, 0, 0, 0);

  // Output quantities.
  real3 norm;
  real3 pt1;
  real3 pt2;
  real depth;
  real eff_rad;
  int nC;

  real oosqrt2 = sqrt(0.5);  // 1/sqrt(2)

  {
    cout << "  sphere center on capsule axis" << endl;
    shapeS.A = real3(3.0, 0.0, 0.0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res || nC != 0) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  cap interaction (separated)" << endl;
    shapeS.A = real3(5.0, 1.5, 0.0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res || nC != 0) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  cap interaction (penetrated)" << endl;
    shapeS.A = real3(5.0, 1.0, 0.0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    StrictEqual(nC, 1);
    WeakEqual(norm, real3(oosqrt2, oosqrt2, 0), precision);
    WeakEqual(depth, sqrt(2.0) - 1.5, precision);
    WeakEqual(pt1, real3(4.0 + 0.5 * oosqrt2, 0.5 * oosqrt2, 0), precision);
    WeakEqual(pt2, real3(5.0 - oosqrt2, 1.0 - oosqrt2, 0), precision);
    WeakEqual(eff_rad, s_rad * c_rad / (s_rad + c_rad), precision);
  }

  {
    cout << "  side interaction (separated)" << endl;
    shapeS.A = real3(2.5, 2.0, 0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res || nC != 0) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  side interaction (penetrated)" << endl;
    shapeS.A = real3(2.5, 1.25, 0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    StrictEqual(nC, 1);
    WeakEqual(norm, real3(0, 1, 0), precision);
    WeakEqual(depth, -0.25, precision);
    WeakEqual(pt1, real3(2.5, 0.5, 0), precision);
    WeakEqual(pt2, real3(2.5, 0.25, 0), precision);
    WeakEqual(eff_rad, s_rad * c_rad / (s_rad + c_rad), precision);
  }
}

// -----------------------------------------------------------------------------

void test_cylinder_sphere() {
  cout << "cylinder_sphere" << endl;

  // Cylinder position and orientation fixed for all tests.
  // Aligned with X axis and shifted by its half-length in the X direction.
  real c_rad = 2.0;
  real c_hlen = 1.5;
  real3 c_pos(c_hlen, 0, 0);
  real4 c_rot = ToReal4(Q_from_AngAxis(CH_C_PI_2, ChVector<>(0, 0, 1)));

  ConvexShape shapeC;
  shapeC.type = ShapeType::CYLINDER;
  shapeC.A = c_pos;
  shapeC.B = real3(c_rad, c_hlen, c_rad);
  shapeC.C = real3(0);
  shapeC.R = c_rot;

  // Sphere position changes for each test.
  real s_rad = 1.0;  // sphere radius

  ConvexShape shapeS;
  shapeS.type = ShapeType::SPHERE;
  shapeS.B = real3(s_rad, 0, 0);
  shapeS.C = real3(0);
  shapeS.R = real4(1, 0, 0, 0);

  // Output quantities.
  real3 norm;
  real3 pt1;
  real3 pt2;
  real depth;
  real eff_rad;
  int nC;

  real oosqrt2 = sqrt(0.5);  // 1/sqrt(2)

  {
    cout << "  sphere center inside cylinder" << endl;
    shapeS.A = real3(2.5, 1.5, 0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res || nC != 0) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  cap interaction (separated)" << endl;
    shapeS.A = real3(4.5, 1.5, 0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res || nC != 0) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  cap interaction (penetrated)" << endl;
    shapeS.A = real3(3.75, 1.5, 0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    StrictEqual(nC, 1);
    WeakEqual(norm, real3(1, 0, 0), precision);
    WeakEqual(depth, -0.25, precision);
    WeakEqual(pt1, real3(3, 1.5, 0), precision);
    WeakEqual(pt2, real3(2.75, 1.5, 0), precision);
    WeakEqual(eff_rad, s_rad, precision);
  }

  {
    cout << "  side interaction (separated)" << endl;
    shapeS.A = real3(2.5, 3.5, 0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res || nC != 0) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  side interaction (penetrated)" << endl;
    shapeS.A = real3(2.5, 2.5, 0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    StrictEqual(nC, 1);
    WeakEqual(norm, real3(0, 1, 0), precision);
    WeakEqual(depth, -0.5, precision);
    WeakEqual(pt1, real3(2.5, 2.0, 0), precision);
    WeakEqual(pt2, real3(2.5, 1.5, 0), precision);
    WeakEqual(eff_rad, s_rad * c_rad / (s_rad + c_rad), precision);
  }

  {
    cout << "  edge interaction (separated)" << endl;
    shapeS.A = real3(4, 3, 0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res || nC != 0) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  edge interaction (penetrated)" << endl;
    shapeS.A = real3(3.5, 2.5, 0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    StrictEqual(nC, 1);
    WeakEqual(norm, real3(oosqrt2, oosqrt2, 0), precision);
    WeakEqual(depth, -1 + oosqrt2, precision);
    WeakEqual(pt1, real3(3.0, 2.0, 0), precision);
    WeakEqual(pt2, real3(3.5 - oosqrt2, 2.5 - oosqrt2, 0), precision);
    WeakEqual(eff_rad, s_rad * edge_radius / (s_rad + edge_radius), precision);
  }
}

// -----------------------------------------------------------------------------

void test_roundedcyl_sphere() {
  cout << "roundedcyl_sphere" << endl;

  // Rounded cylinder position and orientation fixed for all tests.
  // Aligned with X axis and shifted by its half-length in the X direction.
  real c_rad = 2.0;   // radius of skeleton cylinder
  real c_hlen = 1.5;  // half-length of skeleton cylinder
  real c_srad = 0.1;  // radius of sweeping sphere
  real3 c_pos(c_hlen, 0, 0);
  real4 c_rot = ToReal4(Q_from_AngAxis(CH_C_PI_2, ChVector<>(0, 0, 1)));

  ConvexShape shapeC;
  shapeC.type = ShapeType::ROUNDEDCYL;
  shapeC.A = c_pos;
  shapeC.B = real3(c_rad, c_hlen, c_rad);
  shapeC.C = real3(c_srad, 0, 0);
  shapeC.R = c_rot;

  // Sphere position changes for each test.
  real s_rad = 1.0;  // sphere radius

  ConvexShape shapeS;
  shapeS.type = ShapeType::SPHERE;
  shapeS.B = real3(s_rad, 0, 0);
  shapeS.C = real3(0);
  shapeS.R = real4(1, 0, 0, 0);

  // Output quantities.
  real3 norm;
  real3 pt1;
  real3 pt2;
  real depth;
  real eff_rad;
  int nC;

  real oosqrt2 = sqrt(0.5);  // 1/sqrt(2)

  {
    cout << "  sphere center inside cylinder" << endl;
    shapeS.A = real3(2.5, 1.5, 0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res || nC != 0) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  cap interaction (separated)" << endl;
    shapeS.A = real3(4.5, 1.5, 0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res || nC != 0) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  cap interaction (penetrated)" << endl;
    shapeS.A = real3(3.75, 1.5, 0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    StrictEqual(nC, 1);
    WeakEqual(norm, real3(1, 0, 0), precision);
    WeakEqual(depth, -0.35, precision);
    WeakEqual(pt1, real3(3.1, 1.5, 0), precision);
    WeakEqual(pt2, real3(2.75, 1.5, 0), precision);
    WeakEqual(eff_rad, s_rad, precision);
  }

  {
    cout << "  side interaction (separated)" << endl;
    shapeS.A = real3(2.5, 3.5, 0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res || nC != 0) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  side interaction (penetrated)" << endl;
    shapeS.A = real3(2.5, 2.5, 0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    StrictEqual(nC, 1);
    WeakEqual(norm, real3(0, 1, 0), precision);
    WeakEqual(depth, -0.6, precision);
    WeakEqual(pt1, real3(2.5, 2.1, 0), precision);
    WeakEqual(pt2, real3(2.5, 1.5, 0), precision);
    WeakEqual(eff_rad, s_rad * c_rad / (s_rad + c_rad), precision);
  }

  {
    cout << "  edge interaction (separated)" << endl;
    shapeS.A = real3(4, 3, 0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res || nC != 0) {
      cout << "    test failed" << endl;
      exit(1);
    }
  }

  {
    cout << "  edge interaction (penetrated)" << endl;
    shapeS.A = real3(3.5, 2.5, 0);
    bool res = RCollision(shapeC, shapeS, 0, &norm, &pt1, &pt2, &depth, &eff_rad, nC);
    if (!res) {
      cout << "    test failed" << endl;
      exit(1);
    }
    StrictEqual(nC, 1);
    WeakEqual(norm, real3(oosqrt2, oosqrt2, 0), precision);
    WeakEqual(depth, -1.1 + oosqrt2, precision);
    WeakEqual(pt1, real3(3.0 + 0.1 * oosqrt2, 2.0 + 0.1 * oosqrt2, 0), precision);
    WeakEqual(pt2, real3(3.5 - oosqrt2, 2.5 - oosqrt2, 0), precision);
    WeakEqual(eff_rad, s_rad * c_srad / (s_rad + c_srad), precision);
  }
}

// =============================================================================

int main() {
  // Utility functions
  test_snap_to_box();
  test_snap_to_cylinder();

  // Collision detection
  cout << endl << "No separation distance" << endl;
  test_sphere_sphere(false);
  test_box_sphere(false);
  test_capsule_sphere();
  test_cylinder_sphere();
  test_roundedcyl_sphere();

  cout << endl << "With separation distance" << endl;
  test_sphere_sphere(true);
  test_box_sphere(true);

  return 0;
}
