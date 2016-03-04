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
// Description: definition of a real number which can be defined as a float
// (increased speed on some architectures) or a double (increased precision)
// =============================================================================

#pragma once

#include "chrono_parallel/ChParallelDefines.h"
#include <cmath>
#include <float.h>

namespace chrono {

// If the user specified using doubles, define the real type as double
// Also set some constants. The same is done if floats were specified.
#ifdef CHRONO_PARALLEL_USE_DOUBLE
typedef double real;
#define C_LARGE_REAL DBL_MAX
#define C_EPSILON DBL_EPSILON
#else
typedef float real;
#define C_LARGE_REAL FLT_MAX
#define C_EPSILON FLT_EPSILON
#endif

//#define C_Pi 4 * ATan(real(1.0))
//#define C_2Pi real(2.0) * C_Pi
//#define C_InvPi real(1.0) / C_Pi
//#define C_Inv2Pi real(0.5) / C_Pi
//
//#define C_DegToRad C_Pi / real(180.0)
//#define C_RadToDeg real(180.0 / C_Pi

#ifdef CHRONO_PARALLEL_USE_DOUBLE

// Trig Functions
// ========================================================================================
CUDA_HOST_DEVICE static inline real Sin(const real theta) {
    return sin(theta);
}
CUDA_HOST_DEVICE static inline real Cos(const real theta) {
    return cos(theta);
}
CUDA_HOST_DEVICE static inline real Tan(const real theta) {
    return tan(theta);
}
CUDA_HOST_DEVICE static inline real ASin(const real theta) {
    return asin(theta);
}
CUDA_HOST_DEVICE static inline real ACos(const real theta) {
    return acos(theta);
}
CUDA_HOST_DEVICE static inline real ATan(const real theta) {
    return atan(theta);
}
CUDA_HOST_DEVICE static inline real ATan2(const real x, const real y) {
    return atan2(x, y);
}
// Geometric Functions
// ========================================================================================
CUDA_HOST_DEVICE static inline real Sqr(const real x) {
    return x * x;
}
CUDA_HOST_DEVICE static inline real Cube(const real x) {
    return x * x * x;
}
CUDA_HOST_DEVICE static inline real Sqrt(const real x) {
    return sqrt(x);
}
CUDA_HOST_DEVICE static inline real InvSqrt(const real x) {
    return 1.0 / sqrt(x);  // could also use rsqrtf(x) here and avoid division
}
CUDA_HOST_DEVICE static inline real Abs(const real x) {
    return fabs(x);
}
CUDA_HOST_DEVICE static inline real Pow(const real b, const real e) {
    return pow(b, e);
}
CUDA_HOST_DEVICE static inline real Mod(const real x, const real y) {
    return fmod(x, y);
}
CUDA_HOST_DEVICE static inline real Exp(const real x) {
    return exp(x);
}
CUDA_HOST_DEVICE static inline real Min(const real a, const real b) {
    return fmin(a, b);
}
CUDA_HOST_DEVICE static inline real Max(const real a, const real b) {
    return fmax(a, b);
}
CUDA_HOST_DEVICE static inline real Floor(const real a) {
    return floor(a);
}
CUDA_HOST_DEVICE static inline real Ceil(const real a) {
    return ceil(a);
}
CUDA_HOST_DEVICE static inline real Round(const real a) {
    return round(a);
}
CUDA_HOST_DEVICE static inline real Log(const real a) {
    return log(a);
}
#else

// Trig Functions
// ========================================================================================
CUDA_HOST_DEVICE static inline real Sin(const real theta) {
    return sinf(theta);
}
CUDA_HOST_DEVICE static inline real Cos(const real theta) {
    return cosf(theta);
}
CUDA_HOST_DEVICE static inline real Tan(const real theta) {
    return tanf(theta);
}
CUDA_HOST_DEVICE static inline real ASin(const real theta) {
    return asinf(theta);
}
CUDA_HOST_DEVICE static inline real ACos(const real theta) {
    return acosf(theta);
}
CUDA_HOST_DEVICE static inline real ATan(const real theta) {
    return atanf(theta);
}
CUDA_HOST_DEVICE static inline real ATan2(const real x, const real y) {
    return atan2f(x, y);
}
// CUDA_HOST_DEVICE static inline real DegToRad(const real t) {
//    return t * C_DegToRad;
//}
// CUDA_HOST_DEVICE static inline real RadToDeg(const real t) {
//    return t * C_RadToDeg;
//}

// Geometric Functions
// ========================================================================================
CUDA_HOST_DEVICE static inline real Sqr(const real x) {
    return x * x;
}
CUDA_HOST_DEVICE static inline real Cube(const real x) {
    return x * x * x;
}
CUDA_HOST_DEVICE static inline real Sqrt(const real x) {
    return sqrtf(x);
}
CUDA_HOST_DEVICE static inline real InvSqrt(const real x) {
    return 1.0f / sqrtf(x);  // could also use rsqrtf(x) here and avoid division
}
CUDA_HOST_DEVICE static inline real Abs(const real x) {
    return fabsf(x);
}
CUDA_HOST_DEVICE static inline real Pow(const real b, const real e) {
    return powf(b, e);
}
CUDA_HOST_DEVICE static inline real Mod(const real x, const real y) {
    return fmod(x, y);
}
CUDA_HOST_DEVICE static inline real Exp(const real x) {
    return expf(x);
}
CUDA_HOST_DEVICE static inline real Min(const real a, const real b) {
    return fminf(a, b);
}
CUDA_HOST_DEVICE static inline real Max(const real a, const real b) {
    return fmaxf(a, b);
}
CUDA_HOST_DEVICE static inline real Floor(const real a) {
    return floorf(a);
}
CUDA_HOST_DEVICE static inline real Ceil(const real a) {
    return ceilf(a);
}
CUDA_HOST_DEVICE static inline real Round(const real a) {
    return roundf(a);
}
CUDA_HOST_DEVICE static inline real Log(const real a) {
    return logf(a);
}
#endif

// CUDA_HOST_DEVICE static inline real DegToRad(real t) {
//    return t * C_DegToRad;
//}
// CUDA_HOST_DEVICE static inline real RadToDeg(real t) {
//    return t * C_RadToDeg;
//}

// CUDA_HOST_DEVICE static inline real Sign(const real x) {
//  return x < real(0.0) ? -1.0f : 1.0f;
//}
// Returns a -1 if the value is negative
// Returns a +1 if the value is positive
// Otherwise returns zero, this should only happen if the given value is zero
CUDA_HOST_DEVICE static inline real Sign(const real& x) {
    if (x < 0) {
        return -1;
    } else if (x > 0) {
        return 1;
    } else {
        return 0;
    }
}

// Checks if the value is zero to within a certain epsilon
// in this case ZERO_EPSILON is defined based on what the base type of real is
CUDA_HOST_DEVICE static inline bool IsZero(const real x) {
    return Abs(x) < C_EPSILON;
}

// template <typename T>
// inline T Min(T a, T b) {
//    return a < b ? a : b;
//}
//
// template <typename T>
// inline T Max(T a, T b) {
//    return a > b ? a : b;
//}

// Check if two values are equal using a small delta/epsilon value.
// Essentially a fuzzy comparison operator
// template <typename T>
// inline bool IsEqual(const T& x, const T& y) {
//    return IsZero(x - y);
//}

// Check if two values are equal using a small delta/epsilon value.
// Essentially a fuzzy comparison operator
template <typename T>
CUDA_HOST_DEVICE static inline bool IsEqual(const T& _a, const T& _b) {
    real ab;
    ab = Abs(_a - _b);
    if (Abs(ab) < C_EPSILON)
        return 1;
    real a, b;
    a = Abs(_a);
    b = Abs(_b);
    if (b > a) {
        return ab < C_EPSILON * b;
    } else {
        return ab < C_EPSILON * a;
    }
}

CUDA_HOST_DEVICE inline real Lerp(const real& start, const real& end, const real& t) {
    return start + (end - start) * t;
}

template <typename T>
CUDA_HOST_DEVICE inline void Swap(T& a, T& b) {
    T temp = a;
    a = b;
    b = temp;
}

template <typename T>
CUDA_HOST_DEVICE void SwapIfGreater(T& a, T& b) {
    if (a > b) {
        Swap(a, b);
    }
}

// Clamps a given value a between user specified minimum and maximum values
CUDA_HOST_DEVICE inline real Clamp(real x, real low, real high) {
    if (low > high) {
        Swap(low, high);
    }
    return Max(low, Min(x, high));
}

CUDA_HOST_DEVICE inline real ClampMin(real x, real low) {
    return Max(low, x);
}

CUDA_HOST_DEVICE inline real ClampMax(real x, real high) {
    return Min(x, high);
}
}
//=========MACROS
#define OPERATOR_EQUALS(op, tin, tout)       \
    inline tout& operator op##=(tin scale) { \
        *this = *this op scale;              \
        return *this;                        \
    }
#define OPERATOR_EQUALSALT(op, tin, tout)                            \
    static inline tout& operator op##=(tout & a, const tin& scale) { \
        a = a op scale;                                              \
        return a;                                                    \
    }
