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
    inline real4() {}
    inline real4(real a) : array{a, a, a, a} {}
    inline real4(real a, real b, real c, real d) : array{a, b, c, d} {}
    inline real4(const real3& v, real w) : array{v.x, v.y, v.y, w} {}
    inline real4(const real4& v) : array{v.x, v.y, v.y, v.w} {}

    inline real operator[](unsigned int i) const { return array[i]; }
    inline real& operator[](unsigned int i) { return array[i]; }
    inline operator real*() { return &array[0]; }
    inline operator const real*() const { return &array[0]; };
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
#elif defined(USE_SSE)
    inline real4(__m128 m) { _mm_storeu_ps(&array[0], m); }
    inline operator __m128() const { return _mm_loadu_ps(&array[0]); }
    inline real4& operator=(const __m128& rhs) {
        _mm_storeu_ps(&array[0], rhs);
        return *this;  // Return a reference to myself.
    }
#endif

    // ========================================================================================

    union {
        real array[4];
        struct {
            real x, y, z, w;
        };
    };
};

real4 operator+(const real4& a, const real4& b);
real4 operator-(const real4& a, const real4& b);
real4 operator*(const real4& a, const real4& b);
real4 operator/(const real4& a, const real4& b);

real4 operator+(const real4& a, real b);
real4 operator-(const real4& a, real b);
real4 operator*(const real4& a, real b);
real4 operator/(const real4& a, real b);

OPERATOR_EQUALSALT(*, real, real4)
OPERATOR_EQUALSALT(/, real, real4)
OPERATOR_EQUALSALT(+, real, real4)
OPERATOR_EQUALSALT(-, real, real4)

OPERATOR_EQUALSALT(*, real4, real4)
OPERATOR_EQUALSALT(/, real4, real4)
OPERATOR_EQUALSALT(+, real4, real4)
OPERATOR_EQUALSALT(-, real4, real4)

real4 operator-(const real4& a);

// Quaternion Class
// ========================================================================================
class quaternion {
  public:
    inline quaternion() {}
    inline quaternion(real a) : array{a, a, a, a} {}
    inline quaternion(real _w, real _x, real _y, real _z) : array{_w, _x, _y, _z} {}
    inline quaternion(const real3& v, real w) : array{v.w, v.x, v.y, v.z} {}
    inline real operator[](unsigned int i) const { return array[i]; }
    inline real& operator[](unsigned int i) { return array[i]; }
    inline operator real*() { return &array[0]; }
    inline operator const real*() const { return &array[0]; };
    inline quaternion& operator=(const quaternion& rhs) {
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
#elif defined(USE_SSE)
    inline quaternion(__m128 m) { _mm_storeu_ps(&w, m); }
    inline operator __m128() const { return _mm_loadu_ps(&w); }
    inline quaternion& operator=(const __m128& rhs) {
        _mm_storeu_ps(&w, rhs);
        return *this;  // Return a reference to myself.
    }
#endif
    //
    //    inline quaternion Inv() const {
    //        quaternion temp;
    //        real t1 = w * w + x * x + y * y + z * z;
    //        t1 = 1.0 / t1;
    //        temp.w = t1 * w;
    //        temp[0] = -t1 * x;
    //        temp[1] = -t1 * y;
    //        temp[2] = -t1 * z;
    //        return temp;
    //    }
    union {
        real array[4];
        struct {
            real w, x, y, z;
        };
    };
};

quaternion operator+(const quaternion& a, real b);
quaternion operator-(const quaternion& a, real b);
quaternion operator*(const quaternion& a, real b);
quaternion operator/(const quaternion& a, real b);
quaternion operator~(const quaternion& a);
quaternion Inv(const quaternion& a);
real Dot(const quaternion& v1, const quaternion& v2);
real Dot(const quaternion& v);
quaternion Mult(const quaternion& a, const quaternion& b);
quaternion Normalize(const quaternion& v);
static inline real3 Rotate(const real3& v, const quaternion& q) {
    real3 t = 2 * Cross(q.vect(), v);
    return v + q.w * t + Cross(q.vect(), t);
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

    result[0] = Abs((e0e0 + e1e1) * real(2.0) - real(1.0)) * v[0] + Abs((e1e2 - e0e3) * real(2.0)) * v[1] +
                Abs((e1e3 + e0e2) * real(2.0)) * v[2];
    result[1] = Abs((e1e2 + e0e3) * real(2.0)) * v[0] + Abs((e0e0 + e2e2) * real(2.0) - real(1.0)) * v[1] +
                Abs((e2e3 - e0e1) * real(2.0)) * v[2];
    result[2] = Abs((e1e3 - e0e2) * real(2.0)) * v[0] + Abs((e2e3 + e0e1) * real(2.0)) * v[1] +
                Abs((e0e0 + e3e3) * real(2.0) - real(1.0)) * v[2];
    return result;
}

static inline quaternion Q_from_AngAxis(const real& angle, const real3& axis) {
    quaternion quat;
    real halfang;
    real sinhalf;
    halfang = (angle * 0.5);
    sinhalf = Sin(halfang);
    quat.w = Cos(halfang);
    quat[0] = axis[0] * sinhalf;
    quat[1] = axis[1] * sinhalf;
    quat[2] = axis[2] * sinhalf;
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

    V[0] = (e1e2 - e0e3) * 2;
    V[1] = (e0e0 + e2e2) * 2 - 1;
    V[2] = (e2e3 + e0e1) * 2;

    return V;
}

static void Print(quaternion v, const char* name) {
    printf("%s\n", name);
    printf("%f %f %f %f\n", v.w, v[0], v[1], v[2]);
}
}
