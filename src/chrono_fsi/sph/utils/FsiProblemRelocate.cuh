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
// Device utilities for moving SPH particles and BCE markers external to the solver
//
// =============================================================================

#ifndef UTILS_RELOCATE_CUH
#define UTILS_RELOCATE_CUH

#include "chrono_fsi/sph/physics/FsiDataManager.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_utils
/// @{

void shift_BCE(const Real3& shift, const FsiDataManager::DefaultProperties& props, FsiDataManager& dm);
void shift_SPH(const Real3& shift, const FsiDataManager::DefaultProperties& props, FsiDataManager& dm);

void moveAABB2AABB_SPH(const RealAABB& aabb_src,
                       const RealAABB& aabb_dest,
                       Real spacing,
                       const FsiDataManager::DefaultProperties& props,
                       FsiDataManager& dm);
void moveAABB2AABB_SPH(const RealAABB& aabb_src,
                       const IntAABB& aabb_dest,
                       Real spacing,
                       const FsiDataManager::DefaultProperties& props,
                       FsiDataManager& dm);

/// @} fsisph_utils

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
