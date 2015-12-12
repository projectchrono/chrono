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
// Description: SSE and normal implementation of a 3D vector
// =============================================================================

#pragma once

#include <algorithm>

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real.h"
#include "chrono_parallel/math/real2.h"

namespace chrono {
#ifdef ENABLE_SSE
// http://fastcpp.blogspot.com/2011/03/changing-sign-of-float-values-using-sse.html
static const __m128 SIGNMASK = _mm_castsi128_ps(_mm_set1_epi32(0x80000000));
#endif

class real3 {
  public:
    inline real3() {}
#ifdef ENABLE_SSE

    inline explicit real3(real a) : mmvalue(_mm_set1_ps(a)) {}
    inline real3(real a, real b, real c) : mmvalue(_mm_setr_ps(a, b, c, 0)) {}
    inline real3(__m128 m) : mmvalue(m) {}
    //inline real3(const real* p) : mmvalue(_mm_setr_ps(p[0], p[1], p[2], 0)) {}
    // ========================================================================================

    inline operator real*() { return &x; }
    inline operator const real*() const { return &x; };
    inline operator __m128() const { return mmvalue; }

    // ========================================================================================

    inline void Set(real _x, real _y, real _z) { mmvalue(_mm_setr_ps(_x, _y, _z, 0)); }

    inline real3 operator+(real scale) const { return _mm_add_ps(mmvalue, _mm_set1_ps(scale)); }
    inline real3 operator-(real scale) const { return _mm_sub_ps(mmvalue, _mm_set1_ps(scale)); }
    inline real3 operator*(real scale) const { return _mm_mul_ps(mmvalue, _mm_set1_ps(scale)); }
    inline real3 operator/(real scale) const { return _mm_div_ps(mmvalue, _mm_set1_ps(scale)); }

    inline real3 operator+(const real3& v) const { return _mm_add_ps(mmvalue, v.mmvalue); }
    inline real3 operator-(const real3& v) const { return _mm_sub_ps(mmvalue, v.mmvalue); }
    inline real3 operator*(const real3& v) const { return _mm_mul_ps(mmvalue, v.mmvalue); }
    inline real3 operator/(const real3& v) const { return _mm_div_ps(mmvalue, v.mmvalue); }

    inline real3 operator-() const { return _mm_xor_ps(mmvalue, SIGNMASK); }
    // ========================================================================================

    inline real3 Abs(const real3& v) const { return _mm_andnot_ps(SIGNMASK, v.mmvalue); }

#else

    inline explicit real3(real a) : x(a), y(a), z(a) {}
    inline real3(real _x, real _y, real _z) : x(_x), y(_y), z(_z) {}
    //inline real3(const real* p) : x(p[0]), y(p[1]), z(p[2]) {}
    // ========================================================================================

    inline operator real*() { return &x; }
    inline operator const real*() const { return &x; };
    // ========================================================================================

    inline void Set(real _x, real _y, real _z) {
        x = _x;
        y = _y;
        z = _z;
    }

    inline real3 operator+(real b) const { return real3(x + b, y + b, z + b); }
    inline real3 operator-(real b) const { return real3(x - b, y - b, z - b); }
    inline real3 operator*(real b) const { return real3(x * b, y * b, z * b); }
    inline real3 operator/(real b) const { return real3(x / b, y / b, z / b); }

    inline real3 operator+(const real3& b) const { return real3(x + b.x, y + b.y, z + b.z); }
    inline real3 operator-(const real3& b) const { return real3(x - b.x, y - b.y, z - b.z); }
    inline real3 operator*(const real3& b) const { return real3(x * b.x, y * b.y, z * b.z); }
    inline real3 operator/(const real3& b) const { return real3(x / b.x, y / b.y, z / b.z); }

    inline real3 operator-() const { return real3(-x, -y, -z); }

    inline real3 Abs() const { return real3(fabs(x), fabs(y), fabs(z)); }
// ========================================================================================

#endif

    OPERATOR_EQUALS(*, real, real3)
    OPERATOR_EQUALS(/, real, real3)
    OPERATOR_EQUALS(+, real, real3)
    OPERATOR_EQUALS(-, real, real3)

    OPERATOR_EQUALS(*, real3, real3)
    OPERATOR_EQUALS(/, real3, real3)
    OPERATOR_EQUALS(+, real3, real3)
    OPERATOR_EQUALS(-, real3, real3)

    union {
        struct {
            real x, y, z;
        };
        real array[4];
#ifdef ENABLE_SSE
        __m128 mmvalue;
#endif
    };
};

static real3 operator*(real lhs, const real3& rhs) {
    real3 r(rhs);
    r *= lhs;
    return r;
}

static real3 operator/(real lhs, const real3& rhs) {
    real3 r(rhs);
    r.x = lhs / r.x;
    r.y = lhs / r.y;
    r.z = lhs / r.z;
    return r;
}

static bool operator==(const real3& lhs, const real3& rhs) {
    return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}

static inline real Dot(const real3& v1, const real3& v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

static inline real Dot(const real3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

static inline real3 Sqrt(real3 v) {
    return real3(Sqrt(v.x), Sqrt(v.y), Sqrt(v.z));
}

static inline real Length2(const real3& v1) {
    return v1.x * v1.x + v1.y * v1.y + v1.z * v1.z;
}

static inline real3 Cross(const real3& b, const real3& c) {
    return real3(b.y * c.z - b.z * c.y, b.z * c.x - b.x * c.z, b.x * c.y - b.y * c.x);
}

static inline real Max(const real3& a) {
    return Max(a.x, Max(a.y, a.z));
}

static inline real Min(const real3& a) {
    return Min(a.x, Min(a.y, a.z));
}

static inline real3 Max(const real3& a, const real3& b) {
    return real3(Max(a.x, b.x), Max(a.y, b.y), Max(a.z, b.z));
}

static inline real3 Min(const real3& a, const real3& b) {
    return real3(Min(a.x, b.x), Min(a.y, b.y), Min(a.z, b.z));
}

static inline real3 Max(const real3& a, const real& b) {
    return real3(Max(a.x, b), Max(a.y, b), Max(a.z, b));
}

static inline real3 Min(const real3& a, const real& b) {
    return real3(Min(a.x, b), Min(a.y, b), Min(a.z, b));
}

static inline bool IsZero(const real3& v) {
    return Abs(v.x) < C_EPSILON && Abs(v.y) < C_EPSILON && Abs(v.z) < C_EPSILON;
}

static inline real3 Abs(const real3& v) {
    return v.Abs();
}

static inline real3 Clamp(const real3& v, float max_length) {
    real3 x = v;
    real len_sq = Dot(x, x);
    real inv_len = InvSqrt(len_sq);

    if (len_sq > Sqr(max_length))
        x *= inv_len * max_length;

    return x;
}

static inline real3 OrthogonalVector(const real3& v) {
    real3 abs = Abs(v);
    if (abs.x < abs.y) {
        return abs.x < abs.z ? real3(0, v.z, -v.y) : real3(v.y, -v.x, 0);
    } else {
        return abs.y < abs.z ? real3(-v.z, 0, v.x) : real3(v.y, -v.x, 0);
    }
}

static inline real3 UnitOrthogonalVector(const real3& v) {
    return Normalize(OrthogonalVector(v));
}

static inline void Sort(real& a, real& b, real& c) {
    if (a > b)
        Swap(a, b);
    if (b > c)
        Swap(b, c);
    if (a > b)
        Swap(a, b);
}
static void Print(real3 v, const char* name) {
    printf("%s\n", name);
    printf("%f %f %f\n", v.x, v.y, v.z);
}
}
