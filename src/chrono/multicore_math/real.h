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
// Definition of a real number which can be defined as a float (increased speed
// on some architectures) or a double (increased precision).
// =============================================================================

#pragma once

#include <cmath>
#include <cfloat>
#include <algorithm>

#include "chrono/core/ChApiCE.h"
#include "chrono/ChConfig.h"

namespace chrono {

/// @addtogroup chrono_mc_math
/// @{

// Set `real` type, depending on selected precision.
// Set corresponding constants.
#if defined(USE_COLLISION_DOUBLE)
typedef double real;
    #define CH_REAL_MAX DBL_MAX
    #define CH_REAL_MIN DBL_MIN
    #define CH_REAL_EPSILON DBL_EPSILON
#else
typedef float real;
    #define CH_REAL_MAX FLT_MAX
    #define CH_REAL_MIN FLT_MIN
    #define CH_REAL_EPSILON FLT_EPSILON
#endif

//  static inline real DegToRad(real t) {
//    return t * C_DegToRad;
//}
//  static inline real RadToDeg(real t) {
//    return t * C_RadToDeg;
//}

//  static inline real Sign(const real x) {
//  return x < real(0.0) ? -1.0f : 1.0f;
//}

/// Returns a -1 if the value is negative, a +1 if the value is positive. Otherwise returns zero.
template <typename T>
static inline T Sign(const T& x) {
    if (x < 0) {
        return T(-1);
    } else if (x > 0) {
        return T(1);
    } else {
        return T(0);
    }
}

template <typename T>
static inline T Sqr(const T x) {
    return x * x;
}

template <typename T>
static inline T Cube(const T x) {
    return x * x * x;
}

static inline real Min(const real a, const real b, const real c) {
    return std::min(std::min(a, b), c);
}

static inline real Max(const real a, const real b, const real c) {
    return std::max(std::max(a, b), c);
}

/// Check if the value is zero to within a certain epsilon.
/// ZERO_EPSILON is defined based on what the base type of real.
static inline bool IsZero(const real x) {
    return std::abs(x) < CH_REAL_EPSILON;
}

/// Check if two values are equal using a small delta/epsilon value.
/// Essentially a fuzzy comparison operator
template <typename T>
static inline bool IsEqual(const T& _a, const T& _b) {
    real ab;
    ab = std::abs(_a - _b);
    if (ab < CH_REAL_EPSILON)
        return 1;
    real a, b;
    a = std::abs(_a);
    b = std::abs(_b);
    if (b > a) {
        return ab < CH_REAL_EPSILON * b;
    } else {
        return ab < CH_REAL_EPSILON * a;
    }
}

inline real Lerp(const real& start, const real& end, const real& t) {
    return start + (end - start) * t;
}

template <typename T>
inline void Swap(T& a, T& b) {
    T temp = a;
    a = b;
    b = temp;
}

template <typename T>
void Sort(T& a, T& b, T& c) {
    if (a > b)
        Swap(a, b);
    if (b > c)
        Swap(b, c);
    if (a > b)
        Swap(a, b);
}

template <typename T>
void SwapIfGreater(T& a, T& b) {
    if (a > b) {
        Swap(a, b);
    }
}

/// Clamps a given value a between user specified minimum and maximum values.
inline real Clamp(real x, real low, real high) {
    if (low > high) {
        Swap(low, high);
    }
    if (x < low)
        return low;
    if (x > high)
        return high;
    return x;
}

/// Clamps a given value a between user specified minimum and maximum values.
inline void ClampValue(real& x, real low, real high) {
    if (low > high) {
        Swap(low, high);
    }
    if (x < low)
        x = low;
    else if (x > high)
        x = high;
}

inline real ClampMin(real x, real low) {
    return std::max(low, x);
}

inline real ClampMax(real x, real high) {
    return std::min(x, high);
}

//=========MACROS

#define OPERATOR_EQUALS(op, tin, tout)       \
    inline tout& operator op##=(tin scale) { \
        *this = *this op scale;              \
        return *this;                        \
    }
#define OPERATOR_EQUALSALT_PROTO(op, tin, tout) tout& operator op##=(tout & a, const tin & scale);

#define OPERATOR_EQUALSALT(op, tin, tout)               \
    tout& operator op##=(tout & a, const tin & scale) { \
        a = a op scale;                                 \
        return a;                                       \
    }

#define OPERATOR_EQUALS_PROTO(op, tin, tout) tout& operator op##=(tout & a, const tin & scale);

#define OPERATOR_EQUALS_IMPL(op, tin, tout)             \
    tout& operator op##=(tout & a, const tin & scale) { \
        a = a op scale;                                 \
        return a;                                       \
    }

/// @} chrono_mc_math

}  // end namespace chrono
