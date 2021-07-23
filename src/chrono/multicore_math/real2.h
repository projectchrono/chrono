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
    CUDA_HOST_DEVICE real2() : x(0.0f), y(0.0f) {}
    CUDA_HOST_DEVICE explicit real2(real _x) : x(_x), y(_x) {}
    CUDA_HOST_DEVICE real2(real _x, real _y) : x(_x), y(_y) {}
    // real2(const real* p) : x(p[0]), y(p[1]) {}

    CUDA_HOST_DEVICE operator real*() { return &x; }
    CUDA_HOST_DEVICE operator const real*() const { return &x; };

    CUDA_HOST_DEVICE void Set(real x_, real y_) {
        x = x_;
        y = y_;
    }

    real x;
    real y;
};

CUDA_HOST_DEVICE real2 operator+(const real2& a, real b);
CUDA_HOST_DEVICE real2 operator-(const real2& a, real b);
CUDA_HOST_DEVICE real2 operator*(const real2& a, real b);
CUDA_HOST_DEVICE real2 operator/(const real2& a, real b);
CUDA_HOST_DEVICE real2 operator+(const real2& a, const real2& b);
CUDA_HOST_DEVICE real2 operator-(const real2& a, const real2& b);
CUDA_HOST_DEVICE real2 operator*(const real2& a, const real2& b);
CUDA_HOST_DEVICE real2 operator/(const real2& a, const real2& b);
CUDA_HOST_DEVICE real2 operator-(const real2& a);

CUDA_HOST_DEVICE OPERATOR_EQUALS_PROTO(*, real, real2);
CUDA_HOST_DEVICE OPERATOR_EQUALS_PROTO(/, real, real2);
CUDA_HOST_DEVICE OPERATOR_EQUALS_PROTO(+, real, real2);
CUDA_HOST_DEVICE OPERATOR_EQUALS_PROTO(-, real, real2);

CUDA_HOST_DEVICE OPERATOR_EQUALS_PROTO(*, real2, real2);
CUDA_HOST_DEVICE OPERATOR_EQUALS_PROTO(/, real2, real2);
CUDA_HOST_DEVICE OPERATOR_EQUALS_PROTO(+, real2, real2);
CUDA_HOST_DEVICE OPERATOR_EQUALS_PROTO(-, real2, real2);

CUDA_HOST_DEVICE real2 operator*(real lhs, const real2& rhs);

CUDA_HOST_DEVICE bool operator==(const real2& lhs, const real2& rhs);
CUDA_HOST_DEVICE real2 Max(const real2& a, const real2& b);
CUDA_HOST_DEVICE real2 Min(const real2& a, const real2& b);
CUDA_HOST_DEVICE real Dot(const real2& v1, const real2& v2);
CUDA_HOST_DEVICE real Dot(const real2& v);
CUDA_HOST_DEVICE real Length2(const real2& v1);
CUDA_HOST_DEVICE real2 Normalize(const real2& v1);
CUDA_HOST_DEVICE void Print(real2 v, const char* name);

/// @} chrono_mc_math

}  // end namespace chrono
