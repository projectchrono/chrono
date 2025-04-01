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
// Device ulities for moving SPH particles and BCE markers external to the solver
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

void shiftBCE(const Real3& shift_dist, double spacing, FsiDataManager& dm);
void shiftSPH(const Real3& shift_dist, double spacing, FsiDataManager& dm);

void moveSPH(const FsiDataManager::SelectorFunction& op_select,
             const Real3& aabb_min,
             const Real3& aabb_max,
             double spacing,
             FsiDataManager& dm);

/// @} fsisph_utils

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
