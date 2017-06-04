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

#include "chrono_parallel/ChCudaDefines.h"

#include <cmath>
#include <cfloat>

namespace chrono {

/// @addtogroup parallel_math
/// @{

typedef float real;
#define C_LARGE_REAL FLT_MAX
#define C_EPSILON FLT_EPSILON

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
CUDA_HOST_DEVICE static inline real Tanh(const real theta) {
    return tanhf(theta);
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

/// @} parallel_math

} // end namespace chrono
