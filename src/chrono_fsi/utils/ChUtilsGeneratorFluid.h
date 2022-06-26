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
// Utility class for generating fluid markers.
// =============================================================================

#ifndef CH_FSI_UTILS_GENERATORFLUID_H
#define CH_FSI_UTILS_GENERATORFLUID_H

#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"
#include "chrono_fsi/math/custom_math.h"

namespace chrono {
namespace fsi {
namespace utils {

/// @addtogroup fsi_utils
/// @{

/// Kernel function
Real W3h_Spline(Real d, Real h);

/// Particle mass calculator based on the initial spacing and density
Real massCalculator(Real Kernel_h, Real InitialSpacing, Real rho0);

/// Particle number calculator based on the initial spacing and kernel length
Real IniNeiNum(Real Kernel_h, Real InitialSpacing);

/// Create fluid/granular SPH particles for the simulation.
int2 CreateFluidMarkers(std::shared_ptr<SphMarkerDataH> sphMarkersH,
                        std::shared_ptr<FsiGeneralData> fsiGeneralData,
                        std::shared_ptr<SimParams> paramsH);

/// @} fsi_utils

}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono

#endif
