// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Description: Vectorized implementation of a 4D vector/quaternion
// =============================================================================

#pragma once
#include <cstring>

#include "chrono/multicore_math/simd.h"
#include "chrono/multicore_math/real2.h"
#include "chrono/multicore_math/real3.h"

namespace chrono {

/// @addtogroup chrono_mc_math
/// @{

/// Chrono multicore qudruple (4-dimensional array).
class ChApi real4 {
  public:
    inline real4() {}
    inline real4(real a) {
        array[0] = a;
        array[1] = a;
        array[2] = a;
        array[3] = a;
    }
    inline real4(real a, real b, real c, real d) {
        array[0] = a;
        array[1] = b;
        array[2] = c;
        array[3] = d;
    }
    inline real4(const real3& v, real w) {
        array[0] = v.x;
        array[1] = v.y;
        array[2] = v.z;
        array[3] = w;
    }
    inline real4(const real4& v) {
        array[0] = v.x;
        array[1] = v.y;
        array[2] = v.z;
        array[3] = v.w;
    }

    inline real operator[](unsigned int i) const { return array[i]; }
    inline real& operator[](unsigned int i) { return array[i]; }
    inline operator real*() { return &array[0]; }
    inline operator const real*() const { return &array[0]; }

    inline operator real3() { return real3(x, y, z); }
    inline operator const real3() const { return real3(x, y, z); }

    inline real4& operator=(const real4& rhs) {
        memcpy(array, rhs.array, 4 * sizeof(real));

        return *this;  // Return a reference to myself.
    }

#if defined(USE_AVX)
    inline real4(__m256d m) { _mm256_storeu_pd(&array[0], m); }
    inline operator __m256d() const { return _mm256_loadu_pd(&array[0]); }
    inline real4& operator=(const __m256d& rhs) {
        _mm256_storeu_pd(&array[0], rhs);
        return *this;  // Return a reference to myself.
    }
    static inline __m256d Set(real x) { return _mm256_set1_pd(x); }
    static inline __m256d Set(real x, real y, real z, real w) { return _mm256_setr_pd(x, y, z, w); }
#elif defined(USE_SSE)
    inline real4(__m128 m) { _mm_storeu_ps(&array[0], m); }
    inline operator __m128() const { return _mm_loadu_ps(&array[0]); }
    inline real4& operator=(const __m128& rhs) {
        _mm_storeu_ps(&array[0], rhs);
        return *this;  // Return a reference to myself.
    }
    static inline __m128 Set(real x) { return _mm_set1_ps(x); }
    static inline __m128 Set(real x, real y, real z, real w) { return _mm_setr_ps(x, y, z, w); }
#endif

    // ========================================================================================

    union {
        real array[4];
        struct {
            real x, y, z, w;
        };
    };
};

ChApi real4 Set4(real x);
ChApi real4 Set4(real x, real y, real z, real w);

ChApi real4 operator+(const real4& a, const real4& b);
ChApi real4 operator-(const real4& a, const real4& b);
ChApi real4 operator*(const real4& a, const real4& b);
ChApi real4 operator/(const real4& a, const real4& b);

ChApi real4 operator+(const real4& a, real b);
ChApi real4 operator-(const real4& a, real b);
ChApi real4 operator*(const real4& a, real b);
ChApi real4 operator/(const real4& a, real b);

ChApi OPERATOR_EQUALS_PROTO(*, real, real4);
ChApi OPERATOR_EQUALS_PROTO(/, real, real4);
ChApi OPERATOR_EQUALS_PROTO(+, real, real4);
ChApi OPERATOR_EQUALS_PROTO(-, real, real4);

ChApi OPERATOR_EQUALS_PROTO(*, real4, real4);
ChApi OPERATOR_EQUALS_PROTO(/, real4, real4);
ChApi OPERATOR_EQUALS_PROTO(+, real4, real4);
ChApi OPERATOR_EQUALS_PROTO(-, real4, real4);

ChApi real4 operator-(const real4& a);
ChApi real4 Dot4(const real3& v, const real3& v1, const real3& v2, const real3& v3, const real3& v4);

/// Chrono multicore quaternion class.
class ChApi quaternion {
  public:
    quaternion() {}
    quaternion(real a) {
        array[0] = a;
        array[1] = a;
        array[2] = a;
        array[3] = a;
    }
    quaternion(real _w, real _x, real _y, real _z) {
        array[0] = _w;
        array[1] = _x;
        array[2] = _y;
        array[3] = _z;
    }
    quaternion(const real3& v, real w) {
        array[0] = w;
        array[1] = v.x;
        array[2] = v.y;
        array[3] = v.z;
    }
    real operator[](unsigned int i) const { return array[i]; }
    real& operator[](unsigned int i) { return array[i]; }
    operator real*() { return &array[0]; }
    operator const real*() const { return &array[0]; };
    quaternion& operator=(const quaternion& rhs) {
        memcpy(array, rhs.array, 4 * sizeof(real));
        return *this;  // Return a reference to myself.
    }
    inline real3 vect() const { return real3(x, y, z); }

#if defined(USE_AVX)
    inline quaternion(__m256d m) { _mm256_storeu_pd(&w, m); }
    inline operator __m256d() const { return _mm256_loadu_pd(&w); }
    inline quaternion& operator=(const __m256d& rhs) {
        _mm256_storeu_pd(&w, rhs);
        return *this;  // Return a reference to myself.
    }
    static inline __m256d Set(real x) { return _mm256_set1_pd(x); }
    static inline __m256d Set(real w, real x, real y, real z) { return _mm256_setr_pd(w, x, y, z); }
#elif defined(USE_SSE)
    inline quaternion(__m128 m) { _mm_storeu_ps(&w, m); }
    inline operator __m128() const { return _mm_loadu_ps(&w); }
    inline quaternion& operator=(const __m128& rhs) {
        _mm_storeu_ps(&w, rhs);
        return *this;  // Return a reference to myself.
    }
    static inline __m128 Set(real x) { return _mm_set1_ps(x); }
    static inline __m128 Set(real w, real x, real y, real z) { return _mm_setr_ps(w, x, y, z); }
#else

#endif

    union {
        real array[4];
        struct {
            real w, x, y, z;
        };
    };
};

ChApi quaternion SetQ(real x);
ChApi quaternion SetQ(real w, real x, real y, real z);

ChApi quaternion operator+(const quaternion& a, real b);
ChApi quaternion operator-(const quaternion& a, real b);
ChApi quaternion operator*(const quaternion& a, real b);
ChApi quaternion operator/(const quaternion& a, real b);
ChApi quaternion operator~(const quaternion& a);
ChApi quaternion Inv(const quaternion& a);
ChApi real Dot(const quaternion& v1, const quaternion& v2);
ChApi real Dot(const quaternion& v);
ChApi quaternion Mult(const quaternion& a, const quaternion& b);
ChApi quaternion Normalize(const quaternion& v);
ChApi real3 Rotate(const real3& v, const quaternion& q);
ChApi real3 RotateT(const real3& v, const quaternion& q);

// Rotate a vector with the absolute value of a rotation matrix generated by a quaternion
ChApi real3 AbsRotate(const quaternion& q, const real3& v);
ChApi quaternion QuatFromAngleAxis(const real& angle, const real3& axis);
ChApi real3 AMatU(const quaternion& q);
ChApi real3 AMatV(const quaternion& q);
ChApi real3 AMatW(const quaternion& q);
ChApi void Print(quaternion v, const char* name);

/// @} chrono_mc_math

}  // end namespace chrono
