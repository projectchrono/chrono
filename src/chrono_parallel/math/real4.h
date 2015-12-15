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

#if defined(USE_AVX)
    inline real4(real a) : mmvalue(_mm256_set1_pd(a)) {}
    inline real4(const real4& v) : mmvalue(v) {}
    inline real4(real a, real b, real c, real d) : mmvalue(_mm256_setr_pd(a, b, c, d)) {}
    inline real4(const real3& v, real w) : mmvalue(_mm256_setr_pd(v[0], v[1], v[2], w)) {}
    inline real4(const __m256d& m) : mmvalue(m) {}
    inline operator __m256d() const { return mmvalue; }
    inline real operator[](unsigned int i) const { return array[i]; }
    inline real& operator[](unsigned int i) { return array[i]; }
    inline real4& operator=(const real4& rhs) {
        mmvalue = rhs.mmvalue;
        return *this;  // Return a reference to myself.
    }
#elif defined(USE_SIMD)
    inline explicit real4(real a) : mmvalue(_mm_set1_ps(a)) {}
    inline real4(real a, real b, real c, real d) : mmvalue(_mm_setr_ps(a, b, c, d)) {}
    inline real4(const real3& v, real w) : mmvalue(_mm256_setr_ps(v[0], v[1], v[2], w)) {}
    inline real4(__m128 m) : mmvalue(m) {}
    inline operator __m128() const { return mmvalue; }

#else
    inline explicit real4(real a) : x(a), y(a), z(a) {}
    inline real4(real _x, real _y, real _z, real _w) : x(_x), y(_y), z(_z), w(_w) {}
    inline real4(const real3& v, real w) : x(v[0]), y(v[1]), z(v[2]), w(w) {}
#endif

    // ========================================================================================

    //    operator real*() { return &x; }
    //    operator const real*() const { return &x; };

    //    void Set(real _x, real _y, real _z, real _w) {
    //        x = _x;
    //        y = _y;
    //        z = _z;
    //        w = _w;
    //    }
    real4 operator+(const real4& b) const;
    real4 operator-(const real4& b) const;
    real4 operator*(const real4& b) const;
    real4 operator/(const real4& b) const;

    real4 operator+(real b) const;
    real4 operator-(real b) const;
    real4 operator*(real b) const;
    real4 operator/(real b) const;

    OPERATOR_EQUALS(*, real, real4)
    OPERATOR_EQUALS(/, real, real4)
    OPERATOR_EQUALS(+, real, real4)
    OPERATOR_EQUALS(-, real, real4)

    OPERATOR_EQUALS(*, real4, real4)
    OPERATOR_EQUALS(/, real4, real4)
    OPERATOR_EQUALS(+, real4, real4)
    OPERATOR_EQUALS(-, real4, real4)

    real4 operator-() const;
    union {
#if defined(USE_AVX)
        __m256d mmvalue;
#elif defined(USE_SIMD)
        __m128 mmvalue;
#else
#endif
        real array[4];
        struct {
            real x, y, z, w;
        };
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

    quaternion(const real3& v, real w) : w(w), x(v[0]), y(v[1]), z(v[2]) {}

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
        q[0] = x * scale;
        q[1] = y * scale;
        q[2] = z * scale;
        q.w = w * scale;
        return q;
    }

    quaternion operator/(real scale) const {
        quaternion q;
        q[0] = x / scale;
        q[1] = y / scale;
        q[2] = z / scale;
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
        temp[0] = -t1 * x;
        temp[1] = -t1 * y;
        temp[2] = -t1 * z;
        return temp;
    }

    real x, y, z, w;
};

static inline quaternion operator~(quaternion const& a) {
    return quaternion(a.w, -a[0], -a[1], -a[2]);
}

static inline quaternion Inv(quaternion const& a) {
    return a.Inv();
}

static inline real Dot(const quaternion& v1, const quaternion& v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2] + v1.w * v2.w;
}

static inline real Dot(const quaternion& v) {
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v.w * v.w;
}

static inline quaternion Mult(const quaternion& a, const quaternion& b) {
    quaternion temp;
    temp.w = a.w * b.w - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
    temp[0] = a.w * b[0] + a[0] * b.w - a[2] * b[1] + a[1] * b[2];
    temp[1] = a.w * b[1] + a[1] * b.w + a[2] * b[0] - a[0] * b[2];
    temp[2] = a.w * b[2] + a[2] * b.w - a[1] * b[0] + a[0] * b[1];
    return temp;
}

static inline real3 Rotate(const real3& v, const quaternion& q) {
    real3 t = 2 * Cross(real3(q[0], q[1], q[2]), v);
    return v + q.w * t + Cross(real3(q[0], q[1], q[2]), t);
}

static inline real3 RotateT(const real3& v, const quaternion& q) {
    return Rotate(v, ~q);
}

// Rotate a vector with the absolute value of a rotation matrix generated by a quaternion
static inline real3 AbsRotate(const quaternion& q, const real3& v) {
    real e0e0 = q.w * q.w;
    real e1e1 = q[0] * q[0];
    real e2e2 = q[1] * q[1];
    real e3e3 = q[2] * q[2];
    real e0e1 = q.w * q[0];
    real e0e2 = q.w * q[1];
    real e0e3 = q.w * q[2];
    real e1e2 = q[0] * q[1];
    real e1e3 = q[0] * q[2];
    real e2e3 = q[1] * q[2];

    real3 result;

    result[0] =
        Abs((e0e0 + e1e1) * 2.0f - 1.0f) * v[0] + Abs((e1e2 - e0e3) * 2.0f) * v[1] + Abs((e1e3 + e0e2) * 2.0f) * v[2];
    result[1] =
        Abs((e1e2 + e0e3) * 2.0f) * v[0] + Abs((e0e0 + e2e2) * 2.0f - 1.0f) * v[1] + Abs((e2e3 - e0e1) * 2.0f) * v[2];
    result[2] =
        Abs((e1e3 - e0e2) * 2.0f) * v[0] + Abs((e2e3 + e0e1) * 2.0f) * v[1] + Abs((e0e0 + e3e3) * 2.0f - 1.0f) * v[2];
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
    real e2e2 = q[1] * q[1];
    real e0e1 = q.w * q[0];
    real e0e3 = q.w * q[2];
    real e1e2 = q[0] * q[1];
    real e2e3 = q[1] * q[2];

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
