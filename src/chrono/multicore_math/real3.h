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
// Description: Vectorized implementation of a 3d vector
// =============================================================================

#pragma once

#include "chrono/multicore_math/simd.h"
#include "chrono/multicore_math/real.h"
#include "chrono/multicore_math/real2.h"

namespace chrono {

/// @addtogroup chrono_mc_math
/// @{

/// Chrono::Multicore triplet (3-dimensional vector).
class ChApi real3 {
  public:
    CUDA_HOST_DEVICE inline real3() { array[3] = 0; }
    CUDA_HOST_DEVICE inline explicit real3(real a) {
        array[0] = a;
        array[1] = a;
        array[2] = a;
        array[3] = 0;
    }
    CUDA_HOST_DEVICE inline real3(real a, real b, real c) {
        array[0] = a;
        array[1] = b;
        array[2] = c;
        array[3] = 0;
    }
    CUDA_HOST_DEVICE inline real3(const real3& v) {
        array[0] = v.x;
        array[1] = v.y;
        array[2] = v.z;
        array[3] = 0;
    }
    CUDA_HOST_DEVICE inline real operator[](unsigned int i) const { return array[i]; }
    CUDA_HOST_DEVICE inline real& operator[](unsigned int i) { return array[i]; }
    CUDA_HOST_DEVICE inline real3& operator=(const real3& rhs) {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        w = 0;
        return *this;  // Return a reference to myself.
    }

#if defined(USE_AVX)
    inline real3(__m256d m) { _mm256_storeu_pd(&array[0], m); }
    inline operator __m256d() const { return _mm256_loadu_pd(&array[0]); }
    inline real3& operator=(const __m256d& rhs) {
        _mm256_storeu_pd(&array[0], rhs);
        return *this;
    }
    static inline __m256d Set(real x) { return _mm256_set1_pd(x); }
    static inline __m256d Set(real x, real y, real z) { return _mm256_setr_pd(x, y, z, 0.0); }

#elif defined(USE_SSE)
    inline real3(__m128 m) { _mm_storeu_ps(&array[0], m); }
    inline operator __m128() const { return _mm_loadu_ps(&array[0]); }
    inline real3& operator=(const __m128& rhs) {
        _mm_storeu_ps(&array[0], rhs);
        return *this;
    }
    static inline __m128 Set(real x) { return _mm_set1_ps(x); }
    static inline __m128 Set(real x, real y, real z) { return _mm_setr_ps(x, y, z, 0.0f); }
#else

#endif

    // ========================================================================================
    union {
        real array[4];
        struct {
            real x, y, z, w;
        };
    };
};

CUDA_HOST_DEVICE ChApi real3 Set3(real x);
CUDA_HOST_DEVICE ChApi real3 Set3(real x, real y, real z);

CUDA_HOST_DEVICE ChApi real3 operator+(const real3& a, real b);
CUDA_HOST_DEVICE ChApi real3 operator-(const real3& a, real b);
CUDA_HOST_DEVICE ChApi real3 operator*(const real3& a, real b);
CUDA_HOST_DEVICE ChApi real3 operator/(const real3& a, real b);

CUDA_HOST_DEVICE ChApi real3 operator+(const real3& a, const real3& b);
CUDA_HOST_DEVICE ChApi real3 operator-(const real3& a, const real3& b);
CUDA_HOST_DEVICE ChApi real3 operator*(const real3& a, const real3& b);
CUDA_HOST_DEVICE ChApi real3 operator/(const real3& a, const real3& b);

CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_PROTO(*, real, real3);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_PROTO(/, real, real3);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_PROTO(+, real, real3);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_PROTO(-, real, real3);

CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_PROTO(*, real3, real3);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_PROTO(/, real3, real3);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_PROTO(+, real3, real3);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_PROTO(-, real3, real3);

CUDA_HOST_DEVICE ChApi real3 operator-(const real3& a);
CUDA_HOST_DEVICE ChApi real3 operator*(real lhs, const real3& rhs);
CUDA_HOST_DEVICE ChApi real3 operator/(real lhs, const real3& rhs);

CUDA_HOST_DEVICE ChApi bool operator<(const real3& lhs, const real3& rhs);
CUDA_HOST_DEVICE ChApi bool operator>(const real3& lhs, const real3& rhs);
CUDA_HOST_DEVICE ChApi bool operator==(const real3& lhs, const real3& rhs);

CUDA_HOST_DEVICE ChApi real3 Cross(const real3& b, const real3& c);
CUDA_HOST_DEVICE ChApi real Dot(const real3& v1, const real3& v2);
CUDA_HOST_DEVICE ChApi real Dot(const real3& v);
CUDA_HOST_DEVICE ChApi real3 Normalize(const real3& v);
CUDA_HOST_DEVICE ChApi real3 Sqrt(const real3& v);
CUDA_HOST_DEVICE ChApi real3 Round(const real3& v);
CUDA_HOST_DEVICE ChApi real Length(const real3& v);
CUDA_HOST_DEVICE ChApi real Length2(const real3& v1);
CUDA_HOST_DEVICE ChApi real SafeLength(const real3& v);
CUDA_HOST_DEVICE ChApi real3 SafeNormalize(const real3& v, const real3& safe = real3(0));
CUDA_HOST_DEVICE ChApi real Max(const real3& a);
CUDA_HOST_DEVICE ChApi real Min(const real3& a);
CUDA_HOST_DEVICE ChApi real3 Max(const real3& a, const real3& b);
CUDA_HOST_DEVICE ChApi real3 Min(const real3& a, const real3& b);
CUDA_HOST_DEVICE ChApi real3 Max(const real3& a, const real& b);
CUDA_HOST_DEVICE ChApi real3 Min(const real3& a, const real& b);
CUDA_HOST_DEVICE ChApi bool IsZero(const real3& v);
CUDA_HOST_DEVICE ChApi real3 Abs(const real3& v);
CUDA_HOST_DEVICE ChApi real3 Sign(const real3& v);
CUDA_HOST_DEVICE ChApi real3 Clamp(const real3& v, real max_length);
CUDA_HOST_DEVICE ChApi real3 Clamp(const real3& a, const real3& clamp_min, const real3& clamp_max);
CUDA_HOST_DEVICE ChApi real3 OrthogonalVector(const real3& v);
CUDA_HOST_DEVICE ChApi real3 UnitOrthogonalVector(const real3& v);
CUDA_HOST_DEVICE ChApi void Sort(real& a, real& b, real& c);
CUDA_HOST_DEVICE ChApi void Print(real3 v, const char* name);

/// @} chrono_mc_math

}  // end namespace chrono
