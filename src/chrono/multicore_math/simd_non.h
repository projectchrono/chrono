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

#include "chrono/multicore_math/types.h"

using namespace chrono;

namespace simd {

inline real3 Add(const real3& a, const real3& b) {
    return real3(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}

inline real3 Sub(const real3& a, const real3& b) {
    return real3(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

inline real3 Mul(const real3& a, const real3& b) {
    return real3(a[0] * b[0], a[1] * b[1], a[2] * b[2]);
}

inline real3 Div3(const real3& a, const real3& b) {
    return real3(a[0] / b[0], a[1] / b[1], a[2] / b[2]);
}

inline real3 Add(const real3& a, real b) {
    return real3(a[0] + b, a[1] + b, a[2] + b);
}

inline real3 Sub(const real3& a, real b) {
    return real3(a[0] - b, a[1] - b, a[2] - b);
}

inline real3 Mul(const real3& a, real b) {
    return real3(a[0] * b, a[1] * b, a[2] * b);
}

inline real3 Div(const real3& a, real b) {
    return real3(a[0] / b, a[1] / b, a[2] / b);
}

inline real3 Negate(const real3& a) {
    return real3(-a[0], -a[1], -a[2]);
}

inline real Dot(const real3& v1, const real3& v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

inline real Dot(const real3& v) {
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

inline real3 SquareRoot(const real3& v) {
    return real3(Sqrt(v[0]), Sqrt(v[1]), Sqrt(v[2]));
}

inline real3 Cross(const real3& a, const real3& b) {
    real3 result;
    result[0] = (a[1] * b[2]) - (a[2] * b[1]);
    result[1] = (a[2] * b[0]) - (a[0] * b[2]);
    result[2] = (a[0] * b[1]) - (a[1] * b[0]);
    return result;
}

//========================================================
//========================================================
//========================================================

inline real4 Add(const real4& a, const real4& b) {
    return real4(a[0] + b[0], a[1] + b[1], a[2] + b[2], a.w + b.w);
}

inline real4 Sub(const real4& a, const real4& b) {
    return real4(a[0] - b[0], a[1] + b[1], a[2] - b[2], a.w - b.w);
}

inline real4 Mul(const real4& a, const real4& b) {
    return real4(a[0] * b[0], a[1] * b[1], a[2] * b[2], a.w * b.w);
}

inline real4 Div(const real4& a, const real4& b) {
    return real4(a[0] / b[0], a[1] / b[1], a[2] / b[2], a.w / b.w);
}

inline real4 Add(const real4& a, const real3& b) {
    return real4(a[0] + b[0], a[1] + b[1], a[2] + b[2], a.w);
}

inline real4 Sub(const real4& a, const real3& b) {
    return real4(a[0] - b[0], a[1] + b[1], a[2] - b[2], a.w);
}

inline real4 Mul(const real4& a, const real3& b) {
    return real4(a[0] * b[0], a[1] * b[1], a[2] * b[2], a.w);
}

inline real4 Div(const real4& a, const real3& b) {
    return real4(a[0] / b[0], a[1] / b[1], a[2] / b[2], a.w);
}

inline real4 Add(const real4& a, real b) {
    return real4(a[0] + b, a[1] + b, a[2] + b, a.w + b);
}

inline real4 Sub(const real4& a, real b) {
    return real4(a[0] - b, a[1] + b, a[2] - b, a.w - b);
}

inline real4 Mul(const real4& a, real b) {
    return real4(a[0] * b, a[1] * b, a[2] * b, a.w * b);
}

inline real4 Div(const real4& a, real b) {
    return real4(a[0] / b, a[1] / b, a[2] / b, a.w / b);
}

inline real4 Negate(const real4& a) {
    return real4(-a[0], -a[1], -a[2], -a.w);
}

inline real HorizontalAdd(const real4& a) {
    return a[0] + a[1] + a[2] + a[3];
}

inline real HorizontalAdd(const real3& a) {
    return a[0] + a[1] + a[2];
}

inline real4 SquareRoot(real4 v) {
    return real4(Sqrt(v[0]), Sqrt(v[1]), Sqrt(v[2]), Sqrt(v[3]));
}

inline real3 Cross3(const real3& a, const real3& b) {
    real3 result;
    result[0] = (a[1] * b[2]) - (a[2] * b[1]);
    result[1] = (a[2] * b[0]) - (a[0] * b[2]);
    result[2] = (a[0] * b[1]) - (a[1] * b[0]);
    return result;
}

inline real3 Abs(const real3& v) {
    return real3(chrono::Abs(v[0]), chrono::Abs(v[1]), chrono::Abs(v[2]));
}

inline real3 Max(const real3& a, const real3& b) {
    return real3(chrono::Max(a[0], b[0]), chrono::Max(a[1], b[1]), chrono::Max(a[2], b[2]));
}

inline real3 Max(const real3& a, real b) {
    return real3(chrono::Max(a[0], b), chrono::Max(a[1], b), chrono::Max(a[2], b));
}

inline real3 Min(const real3& a, const real3& b) {
    return real3(chrono::Min(a[0], b[0]), chrono::Min(a[1], b[1]), chrono::Min(a[2], b[2]));
}

inline real3 Min(const real3& a, real b) {
    return real3(chrono::Min(a[0], b), chrono::Min(a[1], b), chrono::Min(a[2], b));
}

inline real3 Round(const real3& a) {
    return real3(chrono::Round(a[0]), chrono::Round(a[1]), chrono::Round(a[2]));
}

inline bool IsZero(const real3& v, const real& a) {
    return chrono::Abs(v.x) < a && chrono::Abs(v.y) < a && chrono::Abs(v.z) < a;
}

inline real Max3(const real3& a) {
    return chrono::Max(a[0], chrono::Max(a[1], a[2]));
}

inline real Min3(const real3& a) {
    return chrono::Min(a[0], chrono::Min(a[1], a[2]));
}

inline real Dot3(const real3& a, const real3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline real Dot3(const real3& a) {
    return a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
}

inline real4 Dot4(const real3& v, const real3& a, const real3& b, const real3& c, const real3& d) {
    real4 result;
    result.x = Dot3(v, a);
    result.y = Dot3(v, b);
    result.z = Dot3(v, c);
    result.w = Dot3(v, d);
    return result;
}

inline real Dot4(const quaternion& v1, const quaternion& v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w;
}

inline real Dot4(const quaternion& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w;
}

inline quaternion QuatMult(const quaternion& a, const quaternion& b) {
    quaternion temp;
    temp.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    temp.x = a.w * b.x + a.x * b.w - a.z * b.y + a.y * b.z;
    temp.y = a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z;
    temp.z = a.w * b.z + a.z * b.w - a.y * b.x + a.x * b.y;
    return temp;
}

inline quaternion Add(const quaternion& a, const quaternion& b) {
    return quaternion(a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]);
}

inline quaternion Sub(const quaternion& a, const quaternion& b) {
    return quaternion(a[0] - b[0], a[1] + b[1], a[2] - b[2], a[3] - b[3]);
}

inline quaternion Mul(const quaternion& a, const quaternion& b) {
    return quaternion(a[0] * b[0], a[1] * b[1], a[2] * b[2], a[3] * b[3]);
}

inline quaternion Div(const quaternion& a, const quaternion& b) {
    return quaternion(a[0] / b[0], a[1] / b[1], a[2] / b[2], a[3] / b[3]);
}

inline quaternion Negate(const quaternion& a) {
    return quaternion(-a[0], -a[1], -a[2], -a[3]);
}

template <int i0, int i1, int i2, int i3>
quaternion change_sign(const quaternion& a) {
    quaternion q = a;
    if (i0) {
        q.w = -a.w;
    }
    if (i1) {
        q.x = -a.x;
    }
    if (i2) {
        q.y = -a.y;
    }
    if (i3) {
        q.z = -a.z;
    }
    return q;
}

inline quaternion Normalize(const quaternion& a) {
    real length = Sqrt(Dot4(a));
    if (length < C_REAL_EPSILON) {
        return quaternion(1, 0, 0, 0);
    }
    length = 1.0 / length;
    return quaternion(a.w * length, a.x * length, a.y * length, a.z * length);
}

inline vec3 Set(int x) {
    return vec3(x, x, x);
}

inline vec3 Sub(const vec3& a, const vec3& b) {
    return vec3(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

inline vec3 Add(const vec3& a, const vec3& b) {
    return vec3(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}

inline vec3 Max(const vec3& a, const vec3& b) {
    return vec3(chrono::Max(a[0], b[0]), chrono::Max(a[1], b[1]), chrono::Max(a[2], b[2]));
}

inline vec3 Min(const vec3& a, const vec3& b) {
    return vec3(chrono::Min(a[0], b[0]), chrono::Min(a[1], b[1]), chrono::Min(a[2], b[2]));
}

}  // end namespace simd
