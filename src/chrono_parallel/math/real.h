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
// Description: definition of a real number which can be defined as a float
// (increased speed on some architectures) or a double (increased precision)
// =============================================================================

#pragma once

#include "chrono_parallel/ChParallelDefines.h"
#include <cmath>
#include <float.h>

namespace chrono {
// Check if SSE was found in CMake
#ifdef CHRONO_HAS_SSE
// Depending on the SSE variable in CMake include the proper header file for that
// version of sse
#ifdef CHRONO_SSE_1_0
#include <xmmintrin.h>
#elif defined CHRONO_SSE_2_0
#include <emmintrin.h>
#elif defined CHRONO_SSE_3_0
#include <pmmintrin.h>
#elif defined CHRONO_SSE_4_1
#include <smmintrin.h>
#elif defined CHRONO_SSE_4_2
#include <nmmintrin.h>
#endif
#ifndef CHRONO_USE_SIMD
#define CHRONO_USE_SIMD
#endif
#else
#undef CHRONO_USE_SIMD
#endif

// If the user specified using doubles in CMake make sure that SSE is disabled
#ifdef CHRONO_PARALLEL_USE_DOUBLE
#undef CHRONO_USE_SIMD
#endif
// If the user specified using doubles, define the real type as double
// Also set some constants. The same is done if floats were specified.
#ifdef CHRONO_PARALLEL_USE_DOUBLE
typedef double real;
#define LARGE_REAL 1e30
#define ZERO_EPSILON DBL_EPSILON
#else
typedef float real;
#define LARGE_REAL 1e18f
#define ZERO_EPSILON FLT_EPSILON
#endif

#ifdef CHRONO_PARALLEL_USE_DOUBLE
inline real Sin(real theta) {
    return sin(theta);
}
inline real Cos(real theta) {
    return cos(theta);
}
inline real Tan(real theta) {
    return tan(theta);
}
inline real ASin(real theta) {
    return asin(theta);
}
inline real ACos(real theta) {
    return acos(theta);
}
inline real ATan(real theta) {
    return atan(theta);
}
inline real ATan2(real x, real y) {
    return atan2(x, y);
}
inline real Sqrt(real x) {
    return sqrt(x);
}
inline real InvSqrt(real x) {
    return 1.0f / sqrt(x);  // could also use rsqrt(x) here and avoid division
}
inline real Abs(real x) {
    return fabs(x);
}
inline real Pow(real b, real e) {
    return pow(b, e);
}
inline real Mod(real x, real y) {
    return fmod(x, y);
}
inline real Log(real x) {
    return log(x);
}
#else
inline real Sin(real theta) {
    return sinf(theta);
}
inline real Cos(real theta) {
    return cosf(theta);
}
inline real Tan(real theta) {
    return tanf(theta);
}
inline real ASin(real theta) {
    return asinf(theta);
}
inline real ACos(real theta) {
    return acosf(theta);
}
inline real ATan(real theta) {
    return atanf(theta);
}
inline real ATan2(real x, real y) {
    return atan2f(x, y);
}
inline real Sqrt(real x) {
    return sqrtf(x);
}
inline real InvSqrt(real x) {
    return 1.0f / sqrtf(x);  // could also use rsqrtf(x) here and avoid division
}
inline real Abs(real x) {
    return fabsf(x);
}
inline real Pow(real b, real e) {
    return powf(b, e);
}
inline real Mod(real x, real y) {
    return fmodf(x, y);
}
inline real Log(real x) {
    return logf(x);
}
#endif

template <typename T>
inline  T Min(T a, T b) {
    return a < b ? a : b;
}

template <typename T>
inline  T Max(T a, T b) {
    return a > b ? a : b;
}


// Clamps a given value a between user specified minimum and maximum values
static inline real clamp(const real& a, const real& clamp_min, const real& clamp_max) {
    if (a < clamp_min) {
        return clamp_min;
    } else if (a > clamp_max) {
        return clamp_max;
    } else {
        return a;
    }
}
// Performs a linear interpolation between a and b using alpha
static inline real lerp(const real& a, const real& b, const real& alpha) {
    return (a + alpha * (b - a));
}
// Checks if the value is zero to within a certain epsilon
// in this case ZERO_EPSILON is defined based on what the base type of real is
static inline bool IsZero(const real& a) {
    return Abs(a) < ZERO_EPSILON;
}

// Check if two values are equal using a small delta/epsilon value.
// Essentially a fuzzy comparison operator
static inline bool isEqual(const real& _a, const real& _b) {
    real ab;
    ab = Abs(_a - _b);
    if (Abs(ab) < ZERO_EPSILON)
        return 1;
    real a, b;
    a = Abs(_a);
    b = Abs(_b);
    if (b > a) {
        return ab < ZERO_EPSILON * b;
    } else {
        return ab < ZERO_EPSILON * a;
    }
}

// Returns a -1 if the value is negative
// Returns a +1 if the value is positive
// Otherwise returns zero, this should only happen if the given value is zero
static inline real sign(const real& x) {
    if (x < 0) {
        return -1;
    } else if (x > 0) {
        return 1;
    } else {
        return 0;
    }
}


}
