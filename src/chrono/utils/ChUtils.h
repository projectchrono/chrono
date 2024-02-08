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

#include "chrono/core/ChApiCE.h"

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

}  // end namespace chrono

#endif
