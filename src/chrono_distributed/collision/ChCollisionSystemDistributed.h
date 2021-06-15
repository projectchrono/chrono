// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================

#pragma once

#include "chrono_multicore/ChDataManager.h"
#include "chrono_multicore/collision/ChCollisionSystemChronoMulticore.h"

#include "chrono_distributed/ChDistributedDataManager.h"

namespace chrono {
namespace collision {

/// @addtogroup distributed_collision
/// @{

/// This class scaffolds on ChCollisionSystemChronoMulticore in order to manage
/// collision data for the system during MPI exchanges.
/// Maintains a mapping from a body to its shapes.
class ChCollisionSystemDistributed : public ChCollisionSystemChronoMulticore {
  public:
    ChCollisionSystemDistributed(ChMulticoreDataManager* dm, ChDistributedDataManager* ddm);
    ~ChCollisionSystemDistributed();

    /// Add a collision model to the collision engine.
    /// Creates a mapping entry from the associated body to its collision shapes.
    virtual void Add(ChCollisionModel* model) override;

    /// Remove a collision model from the collision engine.
    /// Deactivates the body in the data manager of Chrono::Multicore and marks the space as free.
    virtual void Remove(ChCollisionModel* model) override;

  private:
    /// Mark bodies whose AABB is contained within the specified box.
    virtual void GetOverlappingAABB(custom_vector<char>& active_id, real3 Amin, real3 Amax) override;

    ChDistributedDataManager* ddm;
};
/// @} distributed_collision

} /* namespace collision */
} /* namespace chrono */
