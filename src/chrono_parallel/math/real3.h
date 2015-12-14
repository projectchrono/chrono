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
#include "chrono_parallel/math/sse.h"

namespace chrono {

class real3 {
  public:
#if defined(CHRONO_USE_SIMD) && defined(CHRONO_HAS_AVX) && defined(CHRONO_PARALLEL_USE_DOUBLE)
    inline real3() : mmvalue(_mm256_setzero_pd()) {}
    inline explicit real3(real a) : mmvalue(_mm256_set1_pd(a)) {}
    inline real3(real a, real b, real c) : mmvalue(_mm256_setr_pd(a, b, c, 0)) {}
    inline real3(__m256 m) : mmvalue(m) {}
    inline operator __m256() const { return mmvalue; }

#elif defined(CHRONO_USE_SIMD) && defined(CHRONO_HAS_SSE) && !defined(CHRONO_PARALLEL_USE_DOUBLE)
    inline real3() : mmvalue(_mm_setzero_ps()) {}
    inline explicit real3(real a) : mmvalue(_mm_setr_ps(a, a, a, 0)) {}
    inline real3(real a, real b, real c) : mmvalue(_mm_setr_ps(a, b, c, 0)) {}
    inline real3(__m128 m) : mmvalue(m) {}
    inline operator __m128() const { return mmvalue; }

#else
    inline real3() : x(0), y(0), z(0) {}
    inline explicit real3(real a) : x(a), y(a), z(a) {}
    inline real3(real _x, real _y, real _z) : x(_x), y(_y), z(_z) {}
#endif

    // ========================================================================================

    inline operator real*() { return &array[0]; }
    inline operator const real*() const { return &array[0]; };

    union {
        struct {
            real x, y, z, w;
        };
        real array[4];
#if defined(CHRONO_USE_SIMD) && defined(CHRONO_HAS_AVX) && defined(CHRONO_PARALLEL_USE_DOUBLE)
        __m256 mmvalue;
#elif defined(CHRONO_USE_SIMD) && defined(CHRONO_HAS_SSE) && !defined(CHRONO_PARALLEL_USE_DOUBLE)
        __m128 mmvalue;
#endif
    };
};

real3 operator+(const real3& a, real b);
real3 operator-(const real3& a, real b);
real3 operator*(const real3& a, real b);
real3 operator/(const real3& a, real b);

real3 operator+(const real3& a, const real3& b);
real3 operator-(const real3& a, const real3& b);
real3 operator*(const real3& a, const real3& b);
real3 operator/(const real3& a, const real3& b);

OPERATOR_EQUALSALT(*, real, real3)
OPERATOR_EQUALSALT(/, real, real3)
OPERATOR_EQUALSALT(+, real, real3)
OPERATOR_EQUALSALT(-, real, real3)

OPERATOR_EQUALSALT(*, real3, real3)
OPERATOR_EQUALSALT(/, real3, real3)
OPERATOR_EQUALSALT(+, real3, real3)
OPERATOR_EQUALSALT(-, real3, real3)

real3 operator-(const real3& a);

real3 operator*(real lhs, const real3& rhs);
real3 operator/(real lhs, const real3& rhs);

bool operator==(const real3& lhs, const real3& rhs);

real3 Cross(const real3& b, const real3& c);
real Dot(const real3& v1, const real3& v2);
real Dot(const real3& v);
real3 Sqrt(real3 v);

static inline real Length2(const real3& v1) {
    return Dot(v1);
}

real Max(const real3& a);
real Min(const real3& a);
real3 Max(const real3& a, const real3& b);
real3 Min(const real3& a, const real3& b);
real3 Max(const real3& a, const real& b);
real3 Min(const real3& a, const real& b);

static inline bool IsZero(const real3& v) {
    return Abs(v.mmvalue[0]) < C_EPSILON && Abs(v.mmvalue[1]) < C_EPSILON && Abs(v.mmvalue[2]) < C_EPSILON;
}

real3 Abs(const real3& v);

static inline real3 Clamp(const real3& v, real max_length) {
    real3 x = v;
    real len_sq = Dot(x);
    real inv_len = InvSqrt(len_sq);

    if (len_sq > Sqr(max_length))
        x *= inv_len * max_length;

    return x;
}
real3 Clamp(const real3& a, const real3& clamp_min, const real3& clamp_max);

//
// static inline real3 Clamp(const real3& a, const real3& clamp_min, const real3& clamp_max) {
//    real3 clampv;
//    clampv.x = Clamp(a.x, clamp_min.x, clamp_max.x);
//    clampv.y = Clamp(a.y, clamp_min.y, clamp_max.y);
//    clampv.z = Clamp(a.z, clamp_min.z, clamp_max.z);
//    return clampv;
//}
//
// static inline real3 OrthogonalVector(const real3& v) {
//    real3 abs = Abs(v);
//    if (abs.x < abs.y) {
//        return abs.x < abs.z ? real3(0, v.z, -v.y) : real3(v.y, -v.x, 0);
//    } else {
//        return abs.y < abs.z ? real3(-v.z, 0, v.x) : real3(v.y, -v.x, 0);
//    }
//}
//
// static inline real3 UnitOrthogonalVector(const real3& v) {
//    return Normalize(OrthogonalVector(v));
//}
//
// static inline void Sort(real& a, real& b, real& c) {
//    if (a > b)
//        Swap(a, b);
//    if (b > c)
//        Swap(b, c);
//    if (a > b)
//        Swap(a, b);
//}
static void Print(real3 v, const char* name) {
    printf("%s\n", name);
    printf("%f %f %f\n", v.mmvalue[0], v.mmvalue[1], v.mmvalue[2]);
}
}
