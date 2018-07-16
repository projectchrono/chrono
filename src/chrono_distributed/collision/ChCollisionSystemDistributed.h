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

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/collision/ChCollisionSystemParallel.h"

#include "chrono_distributed/ChDistributedDataManager.h"

namespace chrono {
namespace collision {

/// This class scaffolds on ChCollisionSystemParallel in order to manage
/// collision data for the system during MPI exchanges.
/// Maintains a mapping from a body to its shapes.
class ChCollisionSystemDistributed : public ChCollisionSystemParallel {
  public:
    ChCollisionSystemDistributed(ChParallelDataManager* dm, ChDistributedDataManager* ddm);
    virtual ~ChCollisionSystemDistributed();

    /// Adds the collision model to the collision system and creates
    /// a mapping entry from the associated body to its collision shapes
    virtual void Add(ChCollisionModel* model) override;

    /// Deactivates the body in the data manager of
    /// chrono::parallel and marks the space as free
    virtual void Remove(ChCollisionModel* model) override;

    virtual void GetOverlappingAABB(custom_vector<char>& active_id, real3 Amin, real3 Amax) override;

  protected:
    ChDistributedDataManager* ddm;
};

} /* namespace collision */
} /* namespace chrono */
