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

#pragma once

#include "chrono/multicore_math/real.h"

namespace chrono {

/// @addtogroup chrono_mc_math
/// @{

/// Chrono::Multicore pair (2-dimensional vector).
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

    real x;
    real y;
};

real2 operator+(const real2& a, real b);
real2 operator-(const real2& a, real b);
real2 operator*(const real2& a, real b);
real2 operator/(const real2& a, real b);
real2 operator+(const real2& a, const real2& b);
real2 operator-(const real2& a, const real2& b);
real2 operator*(const real2& a, const real2& b);
real2 operator/(const real2& a, const real2& b);
real2 operator-(const real2& a);

OPERATOR_EQUALS_PROTO(*, real, real2);
OPERATOR_EQUALS_PROTO(/, real, real2);
OPERATOR_EQUALS_PROTO(+, real, real2);
OPERATOR_EQUALS_PROTO(-, real, real2);

OPERATOR_EQUALS_PROTO(*, real2, real2);
OPERATOR_EQUALS_PROTO(/, real2, real2);
OPERATOR_EQUALS_PROTO(+, real2, real2);
OPERATOR_EQUALS_PROTO(-, real2, real2);

real2 operator*(real lhs, const real2& rhs);

bool operator==(const real2& lhs, const real2& rhs);
real2 Max(const real2& a, const real2& b);
real2 Min(const real2& a, const real2& b);
real Dot(const real2& v1, const real2& v2);
real Dot(const real2& v);
real Length2(const real2& v1);
real2 Normalize(const real2& v1);
void Print(real2 v, const char* name);

/// @} chrono_mc_math

}  // end namespace chrono
