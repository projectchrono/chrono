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

#include <cmath>
#include <cfloat>

namespace chrono {

/// @addtogroup chrono_mc_math
/// @{

typedef float real;
#define C_REAL_MAX FLT_MAX
#define C_REAL_MIN FLT_MIN
#define C_REAL_EPSILON FLT_EPSILON

// Trig Functions
// ========================================================================================
static inline real Sin(const real theta) {
    return sinf(theta);
}
static inline real Cos(const real theta) {
    return cosf(theta);
}
static inline real Tan(const real theta) {
    return tanf(theta);
}
static inline real Tanh(const real theta) {
    return tanhf(theta);
}
static inline real ASin(const real theta) {
    return asinf(theta);
}
static inline real ACos(const real theta) {
    return acosf(theta);
}
static inline real ATan(const real theta) {
    return atanf(theta);
}
static inline real ATan2(const real x, const real y) {
    return atan2f(x, y);
}
//  static inline real DegToRad(const real t) {
//    return t * C_DegToRad;
//}
//  static inline real RadToDeg(const real t) {
//    return t * C_RadToDeg;
//}

// Geometric Functions
// ========================================================================================
static inline real Sqrt(const real x) {
    return sqrtf(x);
}
static inline real InvSqrt(const real x) {
    return 1.0f / sqrtf(x);  // could also use rsqrtf(x) here and avoid division
}
static inline real Abs(const real x) {
    return fabsf(x);
}
static inline real Pow(const real b, const real e) {
    return powf(b, e);
}
static inline real Mod(const real x, const real y) {
    return fmod(x, y);
}
static inline real Exp(const real x) {
    return expf(x);
}
static inline real Min(const real a, const real b) {
    return fminf(a, b);
}
static inline real Max(const real a, const real b) {
    return fmaxf(a, b);
}
static inline real Min(const real a, const real b, const real c) {
    return fminf(fminf(a, b), c);
}
static inline real Max(const real a, const real b, const real c) {
    return fmaxf(fmaxf(a, b), c);
}
static inline real Floor(const real a) {
    return floorf(a);
}
static inline real Ceil(const real a) {
    return ceilf(a);
}
static inline real Round(const real a) {
    return roundf(a);
}
static inline real Log(const real a) {
    return logf(a);
}

/// @} chrono_mc_math

}  // end namespace chrono
