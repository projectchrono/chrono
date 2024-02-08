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

#ifndef CHMATHEMATICS_H
#define CHMATHEMATICS_H

#include <cmath>
#include <cfloat>
#include <algorithm>

#include "chrono/core/ChApiCE.h"

namespace chrono {

/// Returns random value in (0..1) interval with Park-Miller method
ChApi double ChRandom();

/// Sets the seed of the ChRandom function 	(Park-Miller method)
ChApi void ChSetRandomSeed(long newseed);

/// Get the current ChRandom seed value.
ChApi long ChGetRandomSeed();

}  // end namespace chrono

#endif
