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
// Description: Vectorized implementation of a 4D vector/Quaternion
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
    CUDA_HOST_DEVICE inline real4() {}
    CUDA_HOST_DEVICE inline real4(real a) {
        array[0] = a;
        array[1] = a;
        array[2] = a;
        array[3] = a;
    }
    CUDA_HOST_DEVICE inline real4(real a, real b, real c, real d) {
        array[0] = a;
        array[1] = b;
        array[2] = c;
        array[3] = d;
    }
    CUDA_HOST_DEVICE inline real4(const real3& v, real w) {
        array[0] = v.x;
        array[1] = v.y;
        array[2] = v.z;
        array[3] = w;
    }
    CUDA_HOST_DEVICE inline real4(const real4& v) {
        array[0] = v.x;
        array[1] = v.y;
        array[2] = v.z;
        array[3] = v.w;
    }

    CUDA_HOST_DEVICE inline real operator[](unsigned int i) const { return array[i]; }
    CUDA_HOST_DEVICE inline real& operator[](unsigned int i) { return array[i]; }
    CUDA_HOST_DEVICE inline operator real*() { return &array[0]; }
    CUDA_HOST_DEVICE inline operator const real*() const { return &array[0]; }

    CUDA_HOST_DEVICE inline operator real3() { return real3(x, y, z); }
    CUDA_HOST_DEVICE inline operator const real3() const { return real3(x, y, z); }

    CUDA_HOST_DEVICE inline real4& operator=(const real4& rhs) {
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

CUDA_HOST_DEVICE ChApi real4 Set4(real x);
CUDA_HOST_DEVICE ChApi real4 Set4(real x, real y, real z, real w);

CUDA_HOST_DEVICE ChApi real4 operator+(const real4& a, const real4& b);
CUDA_HOST_DEVICE ChApi real4 operator-(const real4& a, const real4& b);
CUDA_HOST_DEVICE ChApi real4 operator*(const real4& a, const real4& b);
CUDA_HOST_DEVICE ChApi real4 operator/(const real4& a, const real4& b);

CUDA_HOST_DEVICE ChApi real4 operator+(const real4& a, real b);
CUDA_HOST_DEVICE ChApi real4 operator-(const real4& a, real b);
CUDA_HOST_DEVICE ChApi real4 operator*(const real4& a, real b);
CUDA_HOST_DEVICE ChApi real4 operator/(const real4& a, real b);

CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_PROTO(*, real, real4);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_PROTO(/, real, real4);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_PROTO(+, real, real4);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_PROTO(-, real, real4);

CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_PROTO(*, real4, real4);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_PROTO(/, real4, real4);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_PROTO(+, real4, real4);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_PROTO(-, real4, real4);

CUDA_HOST_DEVICE ChApi real4 operator-(const real4& a);
CUDA_HOST_DEVICE ChApi real4 Dot4(const real3& v, const real3& v1, const real3& v2, const real3& v3, const real3& v4);

/// Chrono multicore quaternion class.
class ChApi quaternion {
  public:
    CUDA_HOST_DEVICE quaternion() {}
    CUDA_HOST_DEVICE quaternion(real a) {
        array[0] = a;
        array[1] = a;
        array[2] = a;
        array[3] = a;
    }
    CUDA_HOST_DEVICE quaternion(real _w, real _x, real _y, real _z) {
        array[0] = _w;
        array[1] = _x;
        array[2] = _y;
        array[3] = _z;
    }
    CUDA_HOST_DEVICE quaternion(const real3& v, real w) {
        array[0] = w;
        array[1] = v.x;
        array[2] = v.y;
        array[3] = v.z;
    }
    CUDA_HOST_DEVICE real operator[](unsigned int i) const { return array[i]; }
    CUDA_HOST_DEVICE real& operator[](unsigned int i) { return array[i]; }
    CUDA_HOST_DEVICE operator real*() { return &array[0]; }
    CUDA_HOST_DEVICE operator const real*() const { return &array[0]; };
    CUDA_HOST_DEVICE quaternion& operator=(const quaternion& rhs) {
        memcpy(array, rhs.array, 4 * sizeof(real));
        return *this;  // Return a reference to myself.
    }
    CUDA_HOST_DEVICE inline real3 vect() const { return real3(x, y, z); }

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

CUDA_HOST_DEVICE ChApi quaternion SetQ(real x);
CUDA_HOST_DEVICE ChApi quaternion SetQ(real w, real x, real y, real z);

CUDA_HOST_DEVICE ChApi quaternion operator+(const quaternion& a, real b);
CUDA_HOST_DEVICE ChApi quaternion operator-(const quaternion& a, real b);
CUDA_HOST_DEVICE ChApi quaternion operator*(const quaternion& a, real b);
CUDA_HOST_DEVICE ChApi quaternion operator/(const quaternion& a, real b);
CUDA_HOST_DEVICE ChApi quaternion operator~(const quaternion& a);
CUDA_HOST_DEVICE ChApi quaternion Inv(const quaternion& a);
CUDA_HOST_DEVICE ChApi real Dot(const quaternion& v1, const quaternion& v2);
CUDA_HOST_DEVICE ChApi real Dot(const quaternion& v);
CUDA_HOST_DEVICE ChApi quaternion Mult(const quaternion& a, const quaternion& b);
CUDA_HOST_DEVICE ChApi quaternion Normalize(const quaternion& v);
CUDA_HOST_DEVICE ChApi real3 Rotate(const real3& v, const quaternion& q);
CUDA_HOST_DEVICE ChApi real3 RotateT(const real3& v, const quaternion& q);

// Rotate a vector with the absolute value of a rotation matrix generated by a quaternion
CUDA_HOST_DEVICE ChApi real3 AbsRotate(const quaternion& q, const real3& v);
CUDA_HOST_DEVICE ChApi quaternion Q_from_AngAxis(const real& angle, const real3& axis);
CUDA_HOST_DEVICE ChApi real3 AMatU(const quaternion& q);
CUDA_HOST_DEVICE ChApi real3 AMatV(const quaternion& q);
CUDA_HOST_DEVICE ChApi real3 AMatW(const quaternion& q);
CUDA_HOST_DEVICE ChApi void Print(quaternion v, const char* name);

/// @} chrono_mc_math

}  // end namespace chrono
