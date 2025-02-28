// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CH_CONSTANTS_H
#define CH_CONSTANTS_H

namespace chrono {

static constexpr double CH_PI = 3.141592653589793238462643383279;
static constexpr double CH_PI_2 = 1.570796326794896619231321691639;
static constexpr double CH_PI_3 = 1.047197551196597631317786181171;
static constexpr double CH_PI_4 = 0.785398163397448309615660845819;
static constexpr double CH_2PI = 6.283185307179586476925286766559;
static constexpr double CH_RAD_TO_DEG = 180.0 / CH_PI;
static constexpr double CH_DEG_TO_RAD = CH_PI / 180.0;
static constexpr double CH_RPM_TO_RAD_S = CH_2PI / 60.0;
static constexpr double CH_RAD_S_TO_RPM = 60.0 / CH_2PI;

static constexpr double CH_SQRT_2 = 1.41421356237309504880;

static constexpr double CH_1_3 = 1.0 / 3.0;
static constexpr double CH_1_6 = 1.0 / 6.0;
static constexpr double CH_2_3 = 2.0 / 3.0;
static constexpr double CH_4_3 = 4.0 / 3.0;

}  // end namespace chrono

#endif
