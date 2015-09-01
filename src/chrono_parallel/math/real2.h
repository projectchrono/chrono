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
// Description: simple operators for a 2D vector
// =============================================================================

#ifndef REAL2_H
#define REAL2_H

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real.h"
#include <iostream>
#define R2 make_real2

namespace chrono {

struct real2 {
  real x, y;
};

static inline real2 make_real2(const real& a, const real& b) {
  real2 t;
  t.x = a;
  t.y = b;
  return t;
}

static inline std::ostream& operator<<(std::ostream& out, const real2& a) {
  out << "[" << a.x << ", " << a.y << "]" << std::endl;
  return out;
}

static inline real2 operator+(const real2& rhs, const real2& lhs) {
  return R2(rhs.x + lhs.x, rhs.y + lhs.y);
}
static inline real2 operator-(const real2& rhs, const real2& lhs) {
  return R2(rhs.x - lhs.x, rhs.y - lhs.y);
}

static inline real2 operator*(const real2& rhs, const real2& lhs) {
  return R2(rhs.x * lhs.x, rhs.y * lhs.y);
}
static inline real2 operator*(const real2& rhs, const real& lhs) {
  return R2(rhs.x * lhs, rhs.y * lhs);
}

static inline real2 operator/(const real2& rhs, const real2& lhs) {
  return R2(rhs.x / lhs.x, rhs.y / lhs.y);
}

static inline real2 operator/(const real2& rhs, const real& lhs) {
  return R2(rhs.x / lhs, rhs.y / lhs);
}

static inline bool operator==(const real2& a, const real2& b) {
  return ((a.x == b.x) && (a.y == b.y));
}
}
#endif
