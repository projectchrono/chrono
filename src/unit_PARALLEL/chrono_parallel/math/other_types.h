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
// Description: definition of other types such as int3 and int2
// =============================================================================

#ifndef OTHER_TYPES_H
#define OTHER_TYPES_H

#include <vector_types.h>

#define S2 _make_short2
#define I3 _make_int3
#define U3 _make_uint3
#define I2 _make_int2

typedef unsigned int uint;

struct bool2 {
  bool x, y;
  bool2() : x(0), y(0) {}
  bool2(bool a, bool b) : x(a), y(b) {}
};

static inline short2 _make_short2(const short& a, const short& b) {
  short2 t;
  t.x = a;
  t.y = b;
  return t;
}

static inline int3 _make_int3(const int& a, const int& b, const int& c) {
  int3 t;
  t.x = a;
  t.y = b;
  t.z = c;
  return t;
}

static inline int2 _make_int2(const int& a, const int& b) {
  int2 t;
  t.x = a;
  t.y = b;
  return t;
}

static inline int3 _make_int3(const real3& a) {
  int3 t;
  t.x = a.x;
  t.y = a.y;
  t.z = a.z;
  return t;
}

static inline uint3 _make_uint3(const real3& a) {
  uint3 t;
  t.x = a.x;
  t.y = a.y;
  t.z = a.z;
  return t;
}

static inline uint3 _make_uint3(const uint& a, const uint& b, const uint& c) {
  uint3 t;
  t.x = a;
  t.y = b;
  t.z = c;
  return t;
}

static inline int3 operator-(const int3& a, const int3& b) {
  return I3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static inline uint3 operator-(const uint3& a, const uint3& b) {
  return U3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static inline int clamp(const int& a, const int& clamp_min, const int& clamp_max) {
  if (a < clamp_min) {
    return clamp_min;
  } else if (a > clamp_max) {
    return clamp_max;
  } else {
    return a;
  }
}

static inline int3 clamp(const int3& a, const int3& clamp_min, const int3& clamp_max) {
  int3 clampv;
  clampv.x = clamp(a.x, clamp_min.x, clamp_max.x);
  clampv.y = clamp(a.y, clamp_min.y, clamp_max.y);
  clampv.z = clamp(a.z, clamp_min.z, clamp_max.z);
  return clampv;
}

static inline std::ostream& operator<<(std::ostream& out, const int2& a) {
  out << "[" << a.x << ", " << a.y << "]" << std::endl;
  return out;
}
static inline std::ostream& operator<<(std::ostream& out, const int3& a) {
  out << "[" << a.x << ", " << a.y << ", " << a.z << "]" << std::endl;
  return out;
}

#endif
