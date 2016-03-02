// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
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
// Description: Implementation of a 2d Vector
// =============================================================================

#pragma once

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real.h"
#include <iostream>

namespace chrono {
class real2 {
  public:
    real2() : x(0.0f), y(0.0f) {}
    explicit real2(real _x) : x(_x), y(_x) {}
    real2(real _x, real _y) : x(_x), y(_y) {}
    // real2(const real* p) : x(p[0]), y(p[1]) {}

    operator real*() { return &x; }
    operator const real*() const { return &x; };

    void Set(real x_, real y_) {
        x = x_;
        y = y_;
    }

    inline real2 operator+(real b) const { return real2(x + b, y + b); }
    inline real2 operator-(real b) const { return real2(x - b, y - b); }
    inline real2 operator*(real b) const { return real2(x * b, y * b); }
    inline real2 operator/(real b) const { return real2(x / b, y / b); }

    inline real2 operator+(const real2& b) const { return real2(x + b.x, y + b.y); }
    inline real2 operator-(const real2& b) const { return real2(x - b.x, y - b.y); }
    inline real2 operator*(const real2& b) const { return real2(x * b.x, y * b.y); }
    inline real2 operator/(const real2& b) const { return real2(x / b.x, y / b.y); }
    inline real2 operator-() const { return real2(-x, -y); }

    OPERATOR_EQUALS(*, real, real2)
    OPERATOR_EQUALS(/, real, real2)
    OPERATOR_EQUALS(+, real, real2)
    OPERATOR_EQUALS(-, real, real2)

    OPERATOR_EQUALS(*, real2, real2)
    OPERATOR_EQUALS(/, real2, real2)
    OPERATOR_EQUALS(+, real2, real2)
    OPERATOR_EQUALS(-, real2, real2)

    real x;
    real y;
};

static real2 operator*(real lhs, const real2& rhs) {
    real2 r(rhs);
    r *= lhs;
    return r;
}

static bool operator==(const real2& lhs, const real2& rhs) {
    return (lhs.x == rhs.x && lhs.y == rhs.y);
}

static real2 Max(const real2& a, const real2& b) {
    return real2(Max(a.x, b.x), Max(a.y, b.y));
}

static real2 Min(const real2& a, const real2& b) {
    return real2(Min(a.x, b.x), Min(a.y, b.y));
}

static inline real Dot(const real2& v1, const real2& v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

static inline real Dot(const real2& v) {
    return v.x * v.x + v.y * v.y;
}

static inline real Length2(const real2& v1) {
    return v1.x * v1.x + v1.y * v1.y;
}
static inline real2 Normalize(const real2& v1) {
    return v1/Sqrt(Dot(v1));
}
static void Print(real2 v, const char* name) {
    printf("%s\n", name);
    printf("%f %f\n", v.x, v.y);
}
}
