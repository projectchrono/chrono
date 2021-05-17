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
// Description: Class definitions for the AABB generator
//
// =============================================================================

#pragma once

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/chrono/ChCollisionData.h"

namespace chrono {
namespace collision {

/// @addtogroup collision_mc
/// @{

/// Generator for Axis-Aligned Bounding Boxes.
class ChApi ChAABBGenerator {
  public:
    ChAABBGenerator();
    void GenerateAABB(real envelope);

    std::shared_ptr<ChCollisionData> data_manager;
};

/// @} collision_mc

}  // end namespace collision
}  // end namespace chrono
