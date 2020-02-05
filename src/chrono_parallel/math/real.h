// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
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

#include "chrono_parallel/ChApiParallel.h"
#include "chrono_parallel/ChConfigParallel.h"
#include "chrono_parallel/ChCudaDefines.h"

#include <cmath>
#include <cfloat>

// If the user specified using doubles, define the real type as double
// Also set some constants. The same is done if floats were specified.
#if defined(CHRONO_PARALLEL_USE_DOUBLE)
#include "chrono_parallel/math/real_double.h"
#else
#include "chrono_parallel/math/real_single.h"
#endif

namespace chrono {

/// @addtogroup parallel_math
/// @{

// CUDA_HOST_DEVICE static inline real DegToRad(real t) {
//    return t * C_DegToRad;
//}
// CUDA_HOST_DEVICE static inline real RadToDeg(real t) {
//    return t * C_RadToDeg;
//}

// CUDA_HOST_DEVICE static inline real Sign(const real x) {
//  return x < real(0.0) ? -1.0f : 1.0f;
//}

/// Returns a -1 if the value is negative.
/// Returns a +1 if the value is positive.
/// Otherwise returns zero, this should only happen if the given value is zero.
template <typename T>
CUDA_HOST_DEVICE static inline T Sign(const T& x) {
    if (x < 0) {
        return T(-1);
    } else if (x > 0) {
        return T(1);
    } else {
        return T(0);
    }
}

template <typename T>
CUDA_HOST_DEVICE static inline T Sqr(const T x) {
    return x * x;
}

template <typename T>
CUDA_HOST_DEVICE static inline T Cube(const T x) {
    return x * x * x;
}

/// Checks if the value is zero to within a certain epsilon
/// in this case ZERO_EPSILON is defined based on what the base type of real is
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

/// Check if two values are equal using a small delta/epsilon value.
/// Essentially a fuzzy comparison operator
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
CUDA_HOST_DEVICE void Sort(T& a, T& b, T& c) {
    if (a > b)
        Swap(a, b);
    if (b > c)
        Swap(b, c);
    if (a > b)
        Swap(a, b);
}

template <typename T>
CUDA_HOST_DEVICE void SwapIfGreater(T& a, T& b) {
    if (a > b) {
        Swap(a, b);
    }
}

/// Clamps a given value a between user specified minimum and maximum values.
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

//=========MACROS
#define OPERATOR_EQUALS(op, tin, tout)       \
    inline tout& operator op##=(tin scale) { \
        *this = *this op scale;              \
        return *this;                        \
    }
#define OPERATOR_EQUALSALT_PROTO(op, tin, tout) tout& operator op##=(tout& a, const tin& scale);

#define OPERATOR_EQUALSALT(op, tin, tout)             \
    tout& operator op##=(tout& a, const tin& scale) { \
        a = a op scale;                               \
        return a;                                     \
    }

#define OPERATOR_EQUALS_PROTO(op, tin, tout) tout& operator op##=(tout& a, const tin& scale);

#define OPERATOR_EQUALS_IMPL(op, tin, tout)           \
    tout& operator op##=(tout& a, const tin& scale) { \
        a = a op scale;                               \
        return a;                                     \
    }

/// @} parallel_math

}  // end namespace chrono
