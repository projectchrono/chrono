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
// Author: Arman Pazouki, Milad Rakhsha
// =============================================================================
//
// Utility class for generating fluid markers.//
// =============================================================================

#ifndef CH_UTILSGENERATORFLUID_H
#define CH_UTILSGENERATORFLUID_H

#include "chrono_fsi/ChSystemFsi_impl.cuh"
#include "chrono_fsi/math/custom_math.h"

namespace chrono {
namespace fsi {
namespace utils {
int2 CreateFluidMarkers(std::shared_ptr<SphMarkerDataH> sphMarkersH,
                        std::shared_ptr<FsiGeneralData> fsiGeneralData,
                        std::shared_ptr<SimParams> paramsH);

}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono

#endif
