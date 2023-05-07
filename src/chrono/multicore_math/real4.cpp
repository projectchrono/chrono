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

#include "chrono/multicore_math/simd.h"
#include "chrono/multicore_math/real4.h"

#if defined(USE_SSE)
    #include "chrono/multicore_math/simd_sse.h"
#elif defined(USE_AVX)
    #include "chrono/multicore_math/simd_avx.h"
#else
    #include "chrono/multicore_math/simd_non.h"
#endif

#include <cstdio>

namespace chrono {

//========================================================
CUDA_HOST_DEVICE ChApi real4 Set4(real x) {
    return real4(x);
}

CUDA_HOST_DEVICE ChApi real4 Set4(real x, real y, real z, real w) {
    return real4(x, y, z, w);
}

CUDA_HOST_DEVICE ChApi real4 operator+(const real4& a, const real4& b) {
    return simd::Add(a, b);
}

CUDA_HOST_DEVICE ChApi real4 operator-(const real4& a, const real4& b) {
    return simd::Sub(a, b);
}

CUDA_HOST_DEVICE ChApi real4 operator*(const real4& a, const real4& b) {
    return simd::Mul(a, b);
}

CUDA_HOST_DEVICE ChApi real4 operator/(const real4& a, const real4& b) {
    return simd::Div(a, b);
}

//========================================================

CUDA_HOST_DEVICE ChApi real4 operator+(const real4& a, real b) {
    return simd::Add(a, Set4(b));
}

CUDA_HOST_DEVICE ChApi real4 operator-(const real4& a, real b) {
    return simd::Sub(a, Set4(b));
}

CUDA_HOST_DEVICE ChApi real4 operator*(const real4& a, real b) {
    return simd::Mul(a, Set4(b));
}

CUDA_HOST_DEVICE ChApi real4 operator/(const real4& a, real b) {
    return simd::Div(a, Set4(b));
}

CUDA_HOST_DEVICE ChApi real4 operator-(const real4& a) {
    return simd::Negate(a);
}

CUDA_HOST_DEVICE ChApi real4 Dot4(const real3& v, const real3& v1, const real3& v2, const real3& v3, const real3& v4) {
    return simd::Dot4(v, v1, v2, v3, v4);
}

CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_IMPL(*, real, real4);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_IMPL(/, real, real4);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_IMPL(+, real, real4);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_IMPL(-, real, real4);

CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_IMPL(*, real4, real4);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_IMPL(/, real4, real4);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_IMPL(+, real4, real4);
CUDA_HOST_DEVICE ChApi OPERATOR_EQUALS_IMPL(-, real4, real4);

//========================================================

CUDA_HOST_DEVICE ChApi quaternion SetQ(real x) {
    return quaternion(x);
}

CUDA_HOST_DEVICE ChApi quaternion SetQ(real w, real x, real y, real z) {
    return quaternion(w, x, y, z);
}

CUDA_HOST_DEVICE ChApi quaternion operator+(const quaternion& a, real b) {
    return simd::Add(a, SetQ(b));
}

CUDA_HOST_DEVICE ChApi quaternion operator-(const quaternion& a, real b) {
    return simd::Sub(a, SetQ(b));
}

CUDA_HOST_DEVICE ChApi quaternion operator*(const quaternion& a, real b) {
    return simd::Mul(a, SetQ(b));
}

CUDA_HOST_DEVICE ChApi quaternion operator/(const quaternion& a, real b) {
    return simd::Div(a, SetQ(b));
}

CUDA_HOST_DEVICE ChApi quaternion operator-(const quaternion& a) {
    return simd::Negate(a);
}

CUDA_HOST_DEVICE ChApi quaternion operator~(const quaternion& a) {
    return simd::change_sign<0, 1, 1, 1>(a);
}

CUDA_HOST_DEVICE ChApi quaternion Inv(const quaternion& a) {
    real t1 = Dot(a);
    return (~a) / t1;
}

CUDA_HOST_DEVICE ChApi real Dot(const quaternion& v1, const quaternion& v2) {
    return simd::Dot4(v1, v2);
}

CUDA_HOST_DEVICE ChApi real Dot(const quaternion& v) {
    return simd::Dot4(v);
}

CUDA_HOST_DEVICE ChApi quaternion Mult(const quaternion& a, const quaternion& b) {
#if defined(CHRONO_AVX_2_0)
    return simd::QuatMult(a, b);
#else
    quaternion temp;
    temp.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    temp.x = a.w * b.x + a.x * b.w - a.z * b.y + a.y * b.z;
    temp.y = a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z;
    temp.z = a.w * b.z + a.z * b.w - a.y * b.x + a.x * b.y;
    return temp;
#endif
}

CUDA_HOST_DEVICE ChApi quaternion Normalize(const quaternion& v) {
    return simd::Normalize(v);
}

CUDA_HOST_DEVICE ChApi real3 Rotate(const real3& v, const quaternion& q) {
    real3 t = 2 * Cross(q.vect(), v);
    return v + q.w * t + Cross(q.vect(), t);
}

CUDA_HOST_DEVICE ChApi real3 RotateT(const real3& v, const quaternion& q) {
    return Rotate(v, ~q);
}

// Rotate a vector with the absolute value of a rotation matrix generated by a quaternion
CUDA_HOST_DEVICE ChApi real3 AbsRotate(const quaternion& q, const real3& v) {
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

CUDA_HOST_DEVICE ChApi quaternion Q_from_AngAxis(const real& angle, const real3& axis) {
    quaternion quat;
    real halfang;
    real sinhalf;
    halfang = (angle * 0.5);
    sinhalf = Sin(halfang);
    quat.w = Cos(halfang);
    quat.x = axis[0] * sinhalf;
    quat.y = axis[1] * sinhalf;
    quat.z = axis[2] * sinhalf;
    return (quat);
}

CUDA_HOST_DEVICE ChApi real3 AMatU(const quaternion& q) {
    real3 V;

    real e0e0 = q.w * q.w;
    real e0e2 = q.w * q.y;
    real e0e3 = q.w * q.z;
    real e1e1 = q.x * q.x;
    real e1e3 = q.x * q.z;
    real e1e2 = q.x * q.y;

    V[0] = (e0e0 + e1e1) * 2 - 1;
    V[1] = (e1e2 + e0e3) * 2;
    V[2] = (e1e3 - e0e2) * 2;

    return V;
}

CUDA_HOST_DEVICE ChApi real3 AMatV(const quaternion& q) {
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

CUDA_HOST_DEVICE ChApi real3 AMatW(const quaternion& q) {
    real3 V;

    real e0e0 = q.w * q.w;
    real e0e1 = q.w * q.x;
    real e0e2 = q.w * q.y;
    real e1e3 = q.x * q.z;
    real e2e3 = q.y * q.z;
    real e3e3 = q.z * q.z;

    V[0] = (e1e3 + e0e2) * 2;
    V[1] = (e2e3 - e0e1) * 2;
    V[2] = (e0e0 + e3e3) * 2 - 1;

    return V;
}

CUDA_HOST_DEVICE ChApi void Print(quaternion v, const char* name) {
    printf("%s\n", name);
    printf("%f %f %f %f\n", v.w, v.x, v.y, v.z);
}

}  // namespace chrono
