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
// Author: Arman Pazouki
// =============================================================================
//
// Utility class for generating fluid markers.//
// =============================================================================

#ifndef CH_UTILSGENERATORFLUID_CUH
#define CH_UTILSGENERATORFLUID_CUH

#include "chrono_fsi/ChFsiDataManager.cuh"
#include "chrono_fsi/custom_math.h"

namespace chrono {
namespace fsi {
namespace utils {
int2 CreateFluidMarkers(SphMarkerDataH &sphMarkersH,
                        FsiGeneralData &fsiGeneralData, SimParams &paramsH);

} // end namespace utils
} // end namespace fsi
} // end namespace chrono

#endif
