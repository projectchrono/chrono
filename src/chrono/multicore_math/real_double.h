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
// Description: definition of a real number which can be defined as a float
// (increased speed on some architectures) or a double (increased precision)
// =============================================================================

#pragma once

#include <cmath>
#include <cfloat>

namespace chrono {

/// @addtogroup chrono_mc_math
/// @{

// If the user specified using doubles, define the real type as double
// Also set some constants. The same is done if floats were specified.
typedef double real;
#define C_LARGE_REAL DBL_MAX
#define C_EPSILON DBL_EPSILON

// Trig Functions
// ========================================================================================
static inline real Sin(const real theta) {
    return sin(theta);
}
static inline real Cos(const real theta) {
    return cos(theta);
}
static inline real Tan(const real theta) {
    return tan(theta);
}
static inline real Tanh(const real theta) {
    return tanh(theta);
}
static inline real ASin(const real theta) {
    return asin(theta);
}
static inline real ACos(const real theta) {
    return acos(theta);
}
static inline real ATan(const real theta) {
    return atan(theta);
}
static inline real ATan2(const real x, const real y) {
    return atan2(x, y);
}
// Geometric Functions
// ========================================================================================
static inline real Sqrt(const real x) {
    return sqrt(x);
}
static inline real InvSqrt(const real x) {
    return 1.0 / sqrt(x);  // could also use rsqrtf(x) here and avoid division
}
static inline real Abs(const real x) {
    return fabs(x);
}
static inline real Pow(const real b, const real e) {
    return pow(b, e);
}
static inline real Mod(const real x, const real y) {
    return fmod(x, y);
}
static inline real Exp(const real x) {
    return exp(x);
}
static inline real Min(const real a, const real b) {
    return fmin(a, b);
}
static inline real Max(const real a, const real b) {
    return fmax(a, b);
}
static inline real Floor(const real a) {
    return floor(a);
}
static inline real Ceil(const real a) {
    return ceil(a);
}
static inline real Round(const real a) {
    return round(a);
}
static inline real Log(const real a) {
    return log(a);
}

/// @} chrono_mc_math

}  // end namespace chrono
