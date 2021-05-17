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
// Description: Implementation of a 2d Vector
// =============================================================================

#include <iostream>

#include "chrono/multicore_math/real2.h"

namespace chrono {

real2 operator+(const real2& a, real b) {
    return real2(a.x + b, a.y + b);
}
real2 operator-(const real2& a, real b) {
    return real2(a.x - b, a.y - b);
}
real2 operator*(const real2& a, real b) {
    return real2(a.x * b, a.y * b);
}
real2 operator/(const real2& a, real b) {
    return real2(a.x / b, a.y / b);
}

real2 operator+(const real2& a, const real2& b) {
    return real2(a.x + b.x, a.y + b.y);
}
real2 operator-(const real2& a, const real2& b) {
    return real2(a.x - b.x, a.y - b.y);
}
real2 operator*(const real2& a, const real2& b) {
    return real2(a.x * b.x, a.y * b.y);
}
real2 operator/(const real2& a, const real2& b) {
    return real2(a.x / b.x, a.y / b.y);
}
real2 operator-(const real2& a) {
    return real2(-a.x, -a.y);
}

OPERATOR_EQUALS_IMPL(*, real, real2);
OPERATOR_EQUALS_IMPL(/, real, real2);
OPERATOR_EQUALS_IMPL(+, real, real2);
OPERATOR_EQUALS_IMPL(-, real, real2);

OPERATOR_EQUALS_IMPL(*, real2, real2);
OPERATOR_EQUALS_IMPL(/, real2, real2);
OPERATOR_EQUALS_IMPL(+, real2, real2);
OPERATOR_EQUALS_IMPL(-, real2, real2);

real2 operator*(real lhs, const real2& rhs) {
    real2 r(rhs);
    r *= lhs;
    return r;
}

bool operator==(const real2& lhs, const real2& rhs) {
    return (lhs.x == rhs.x && lhs.y == rhs.y);
}

real2 Max(const real2& a, const real2& b) {
    return real2(Max(a.x, b.x), Max(a.y, b.y));
}

real2 Min(const real2& a, const real2& b) {
    return real2(Min(a.x, b.x), Min(a.y, b.y));
}

real Dot(const real2& v1, const real2& v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

real Dot(const real2& v) {
    return v.x * v.x + v.y * v.y;
}

real Length2(const real2& v1) {
    return v1.x * v1.x + v1.y * v1.y;
}
real2 Normalize(const real2& v1) {
    return v1 / Sqrt(Dot(v1));
}
void Print(real2 v, const char* name) {
    printf("%s\n", name);
    printf("%f %f\n", v.x, v.y);
}
}  // namespace chrono
