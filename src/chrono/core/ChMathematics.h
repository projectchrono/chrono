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

#ifndef CHMATHEMATICS_H
#define CHMATHEMATICS_H

#include <cmath>
#include <cfloat>
#include <cassert>

#include "chrono/core/ChApiCE.h"

namespace chrono {
// CONSTANTS

static const double CH_C_PI = 3.141592653589793238462643383279;
static const double CH_C_PI_2 = 1.570796326794896619231321691639;
static const double CH_C_PI_4 = 0.785398163397448309615660845819;
static const double CH_C_1_PI = 0.318309886183790671537767526745;
static const double CH_C_2PI = 6.283185307179586476925286766559;
static const double CH_C_RAD_TO_DEG = 180.0 / 3.1415926535897932384626433832795;
static const double CH_C_DEG_TO_RAD = 3.1415926535897932384626433832795 / 180.0;

static const double CH_C_SQRT_2 = 1.41421356237309504880;
static const double CH_C_SQRT_1_2 = 0.70710678118654752440;

static const double CH_C_E = 2.71828182845904523536;
static const double CH_C_LOG2E = 1.44269504088896340736;
static const double CH_C_LOG10E = 0.43429448190325182765;
static const double CH_C_LN2 = 0.69314718055994530941;
static const double CH_C_LN10 = 2.30258509299404568402;

static const double CH_HITOL = 0.001;
static const double CH_LOWTOL = 0.0001;
static const double CH_LOWLOWTOL = 0.00001;

static const double CH_MICROTOL = 1.e-10;
static const double CH_NANOTOL = 1.e-20;

static const double BDF_STEP_HIGH = 0.0001;
static const double BDF_STEP_LOW = 0.0000001;
static const double BDF_STEP_VERYLOW = 1.e-10;
static const double BDF_STEP_TOOLOW = 1.e-20;

// ANGLE CONVERSIONS

/// Computes the atan2, returning angle given cosine and sine.
ChApi double ChAtan2(double mcos, double msin);

// OTHER

/// Returns random value in (0..1) interval with Park-Miller method
ChApi double ChRandom();

/// Sets the seed of the ChRandom function 	(Park-Miller method)
ChApi void ChSetRandomSeed(long newseed);

/// Computes a 1D harmonic multi-octave noise
ChApi double ChNoise(double x, double amp, double freq, int octaves, double amp_ratio);

/// Maximum between two values
inline int ChMax(int a, int b) {
    if (a > b)
        return a;
    return b;
}
/// Maximum between two values
inline double ChMax(double a, double b) {
    if (a > b)
        return a;
    return b;
}
/// Minimum between two values
inline int ChMin(int a, int b) {
    if (a < b)
        return a;
    return b;
}
/// Minimum between two values
inline double ChMin(double a, double b) {
    if (a < b)
        return a;
    return b;
}

/// Clamp and modify the specified value to lie within the given limits.
template <typename T>
void ChClampValue(T& value, T limitMin, T limitMax) {
    if (value < limitMin)
        value = limitMin;
    else if (value > limitMax)
        value = limitMax;
}

/// Clamp the specified value to lie within the given limits.
template <typename T>
T ChClamp(T value, T limitMin, T limitMax) {
    if (value < limitMin)
        return limitMin;
    if (value > limitMax)
        return limitMax;

    return value;
}

/// Signum function.
template <typename T>
int ChSignum(T x) {
    return (x > T(0)) - (x < T(0));
}

/// Parameter make periodic in 0..1
/// (using 0..1 modulus if closed, otherwise clamping in 0..1 range)
ChApi void ChPeriodicPar(double& u, int closed);

}  // end namespace chrono

#endif
