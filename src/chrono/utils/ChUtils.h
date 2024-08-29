// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CH_UTILS_H
#define CH_UTILS_H

#include <algorithm>
#include <cmath>

#include "chrono/core/ChApiCE.h"
#include "chrono/utils/ChConstants.h"

namespace chrono {

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

/// Wrap angle in range.
/// If symmetric, wrap in [-PI; PI).
/// If not symmetric, wrap in [0; 2PI).
template <typename T>
T ChWrapAngle(T angle, bool symmetric = true) {
    T wangle = angle;
    if (symmetric) {  // [-PI; +PI)
        wangle = std::fmod(wangle + CH_PI, CH_2PI);
        wangle < 0 ? wangle += CH_PI : wangle -= CH_PI;
    } else {  // [0; 2PI)
        wangle = std::fmod(wangle, CH_2PI);
        if (wangle < 0)
            wangle += CH_2PI;
    }
    return wangle;
}

}  // end namespace chrono

#endif
