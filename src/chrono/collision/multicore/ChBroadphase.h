// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Description: Class definitions for the Broadpahse
//
// =============================================================================

#pragma once

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/multicore/ChCollisionData.h"

namespace chrono {

/// @addtogroup collision_mc
/// @{

/// Class for performing broad-phase collision detection.
class ChApi ChBroadphase {
  public:
    /// Method for computing grid resolution
    enum class GridType {
        FIXED_RESOLUTION,  ///< user-specified number of bins in each direction
        FIXED_BIN_SIZE,    ///< user-specified grid bin dimension
        FIXED_DENSITY      ///< user-specified density of shapes per bin
    };

    ChBroadphase();

    /// Perform broadphase collision detection.
    /// Collision detection results are loaded in the shared data object (see ChCollisionData).
    void Process();

  private:
    void OneLevelBroadphase();
    void DetermineBoundingBox();
    void OffsetAABB();
    void ComputeTopLevelResolution();
    void RigidBoundingBox();
    void FluidBoundingBox();

    std::shared_ptr<ChCollisionData> cd_data;

    GridType grid_type;    ///< (input) method for setting grid resolution
    vec3 grid_resolution;  ///< (input) number of bins (used for GridType::FIXED_RESOLUTION)
    real3 bin_size;        ///< (input) desired bin dimensions (used for GridType::FIXED_BIN_SIZE)
    real grid_density;     ///< (input) collision grid density (used for GridType::FIXED_DENSITY)

    friend class ChCollisionSystemMulticore;
    friend class ChCollisionSystemChronoMulticore;
};

/// @} collision_mc

}  // end namespace chrono
