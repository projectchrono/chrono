// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Utilities for processing FEA mesh data on the device.
//
// =============================================================================

#ifndef CH_FSI_INTERFACE_CUH
#define CH_FSI_INTERFACE_CUH

#include <thrust/device_vector.h>

#include "chrono_fsi/sph/physics/FsiDataManager.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_physics
/// @{

/// Utility function to calculate (on the device) direction vectors at the flexible 1-D mesh nodes.
/// For 1-D meshes, these are averages of the segment direction vectors of adjacent segments.
void calculateDirectionsMesh1D(FsiDataManager& data_mgr);

/// Utility function to calculate (on the device) direction vectors at the flexible 2-D mesh nodes.
/// For 2-D meshes, these are averages of the face normals of adjacent faces.
void calculateDirectionsMesh2D(FsiDataManager& data_mgr);

void printDirectionsMesh1D(FsiDataManager& data_mgr);

/// @} fsisph_physics

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
