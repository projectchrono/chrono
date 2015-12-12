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
// Description: SSE and normal implementation of a 4D vector/Quaternion
// =============================================================================

#pragma once

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real2.h"
#include "chrono_parallel/math/real3.h"

namespace chrono {

class real4 {
public:
  real4() {}
  explicit real4(real a) : x(a), y(a), z(a), w(a) {}
  real4(real _x, real _y, real _z, real _w) : x(_x), y(_y), z(_z), w(_w) {}
  // real4(const real* p) : x(p[0]), y(p[1]), z(p[2]), w(p[3]) {}

  real4(const real3& v, real w) : x(v.x), y(v.y), z(v.z), w(w) {}

  operator real*() { return &x; }
  operator const real*() const { return &x; };

  void Set(real _x, real _y, real _z, real _w) {
    x = _x;
    y = _y;
    z = _z;
    w = _w;
  }
  inline real4 operator+(const real4& b) const { return real4(x + b.x, y + b.y, z + b.z, w + b.w); }
  inline real4 operator-(const real4& b) const { return real4(x - b.x, y + b.y, z - b.z, w - b.w); }
  inline real4 operator*(const real4& b) const { return real4(x * b.x, y * b.y, z * b.z, w * b.w); }
  inline real4 operator/(const real4& b) const { return real4(x / b.x, y / b.y, z / b.z, w / b.w); }

  inline real4 operator+(const real3& b) const { return real4(x + b.x, y + b.y, z + b.z, w); }
  inline real4 operator-(const real3& b) const { return real4(x - b.x, y + b.y, z - b.z, w); }
  inline real4 operator*(const real3& b) const { return real4(x * b.x, y * b.y, z * b.z, w); }
  inline real4 operator/(const real3& b) const { return real4(x / b.x, y / b.y, z / b.z, w); }

  inline real4 operator+(real b) const { return real4(w + b, x + b, y + b, z + b); }
  inline real4 operator-(real b) const { return real4(w - b, x - b, y + b, z - b); }
  inline real4 operator*(real b) const { return real4(w * b, x * b, y * b, z * b); }
  inline real4 operator/(real b) const { return real4(w / b, x / b, y / b, z / b); }

  OPERATOR_EQUALS(*, real, real4)
  OPERATOR_EQUALS(/, real, real4)
  OPERATOR_EQUALS(+, real, real4)
  OPERATOR_EQUALS(-, real, real4)

  OPERATOR_EQUALS(*, real3, real4)
  OPERATOR_EQUALS(/, real3, real4)
  OPERATOR_EQUALS(+, real3, real4)
  OPERATOR_EQUALS(-, real3, real4)

  OPERATOR_EQUALS(*, real4, real4)
  OPERATOR_EQUALS(/, real4, real4)
  OPERATOR_EQUALS(+, real4, real4)
  OPERATOR_EQUALS(-, real4, real4)

  inline real4 operator-() const { return real4(-x, -y, -z, -w); }

  union {
    struct {
      real x, y, z, w;
    };
#ifdef ENABLE_SSE
    __m128 mmvalue;
#endif
  };
};

// Quaternion Class
// ========================================================================================
class quaternion {
public:
  quaternion() : x(0), y(0), z(0), w(0) {}
  quaternion(real a) : x(a), y(a), z(a), w(a) {}
  quaternion(real _w, real _x, real _y, real _z) : x(_x), y(_y), z(_z), w(_w) {}
  // quaternion(const real* p) : x(p[0]), y(p[1]), z(p[2]), w(p[3]) {}

  quaternion(const real3& v, real w) : x(v.x), y(v.y), z(v.z), w(w) {}

  operator real*() { return &x; }
  operator const real*() const { return &x; };

  void Set(real _w, real _x, real _y, real _z) {
    x = _x;
    y = _y;
    z = _z;
    w = _w;
  }

  quaternion operator*(real scale) const {
    quaternion q;
    q.x = x * scale;
    q.y = y * scale;
    q.z = z * scale;
    q.w = w * scale;
    return q;
  }

  quaternion operator/(real scale) const {
    quaternion q;
    q.x = x / scale;
    q.y = y / scale;
    q.z = z / scale;
    q.w = w / scale;
    return q;
  }

  quaternion& operator*=(real scale) {
    x *= scale;
    y *= scale;
    z *= scale;
    w *= scale;
    return *this;
  }

  quaternion& operator/=(real scale) {
    x /= scale;
    y /= scale;
    z /= scale;
    w /= scale;
    return *this;
  }

  inline quaternion Inv() const {
    quaternion temp;
    real t1 = w * w + x * x + y * y + z * z;
    t1 = 1.0 / t1;
    temp.w = t1 * w;
    temp.x = -t1 * x;
    temp.y = -t1 * y;
    temp.z = -t1 * z;
    return temp;
  }

  real x, y, z, w;
};

static inline quaternion operator~(quaternion const& a) {
  return quaternion(a.w, -a.x, -a.y, -a.z);
}

static inline quaternion Inv(quaternion const& a) {
  return a.Inv();
}

static inline real Dot(const quaternion& v1, const quaternion& v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w;
}

static inline real Dot(const quaternion& v) {
  return v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w;
}

static inline quaternion Mult(const quaternion& a, const quaternion& b) {
  quaternion temp;
  temp.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  temp.x = a.w * b.x + a.x * b.w - a.z * b.y + a.y * b.z;
  temp.y = a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z;
  temp.z = a.w * b.z + a.z * b.w - a.y * b.x + a.x * b.y;
  return temp;
}

static inline real3 Rotate(const real3& v, const quaternion& q) {
  real3 t = 2 * Cross(real3(q.x, q.y, q.z), v);
  return v + q.w * t + Cross(real3(q.x, q.y, q.z), t);
}

static inline real3 RotateT(const real3& v, const quaternion& q) {
  return Rotate(v, ~q);
}

// Rotate a vector with the absolute value of a rotation matrix generated by a quaternion
static inline real3 AbsRotate(const quaternion& q, const real3& v) {
  real e0e0 = q.w * q.w;
  real e1e1 = q.x * q.x;
  real e2e2 = q.y * q.y;
  real e3e3 = q.z * q.z;
  real e0e1 = q.w * q.x;
  real e0e2 = q.w * q.y;
  real e0e3 = q.w * q.z;
  real e1e2 = q.x * q.y;
  real e1e3 = q.x * q.z;
  real e2e3 = q.y * q.z;

  real3 result;

  result.x = Abs((e0e0 + e1e1) * 2.0f - 1.0f) * v.x + Abs((e1e2 - e0e3) * 2.0f) * v.y + Abs((e1e3 + e0e2) * 2.0f) * v.z;
  result.y = Abs((e1e2 + e0e3) * 2.0f) * v.x + Abs((e0e0 + e2e2) * 2.0f - 1.0f) * v.y + Abs((e2e3 - e0e1) * 2.0f) * v.z;
  result.z = Abs((e1e3 - e0e2) * 2.0f) * v.x + Abs((e2e3 + e0e1) * 2.0f) * v.y + Abs((e0e0 + e3e3) * 2.0f - 1.0f) * v.z;
  return result;
}

static inline quaternion Q_from_AngAxis(const real& angle, const real3& axis) {
  quaternion quat;
  real halfang;
  real sinhalf;
  halfang = (angle * 0.5);
  sinhalf = Sin(halfang);
  quat.w = Cos(halfang);
  quat.x = axis.x * sinhalf;
  quat.y = axis.y * sinhalf;
  quat.z = axis.z * sinhalf;
  return (quat);
}

static inline real3 AMatV(const quaternion& q) {
  real3 V;

  real e0e0 = q.w * q.w;
  real e2e2 = q.y * q.y;
  real e0e1 = q.w * q.x;
  real e0e3 = q.w * q.z;
  real e1e2 = q.x * q.y;
  real e2e3 = q.y * q.z;

  V.x = (e1e2 - e0e3) * 2;
  V.y = (e0e0 + e2e2) * 2 - 1;
  V.z = (e2e3 + e0e1) * 2;

  return V;
}

static void Print(quaternion v, const char* name) {
  printf("%s\n", name);
  printf("%f %f %f %f\n", v.w, v.x, v.y, v.z);
}
}
