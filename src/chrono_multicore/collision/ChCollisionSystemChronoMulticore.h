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
// Authors: Radu Serban
// =============================================================================
//
// Chrono custom multicore collision system for Chrono::Multicore.
//
// =============================================================================

#ifndef CH_COLLISION_SYSTEM_CHRONO_MULTICORE_H
#define CH_COLLISION_SYSTEM_CHRONO_MULTICORE_H

#include "chrono_multicore/ChApiMulticore.h"
#include "chrono_multicore/ChDataManager.h"

#include "chrono/collision/multicore/ChCollisionSystemMulticore.h"

namespace chrono {

// Forward declaration for friend class declaration
class ChSystemMulticore;


/// @addtogroup multicore_collision
/// @{

/// Chrono custom multicore collision system.
/// Contains both the broadphase and the narrow phase methods.
class CH_MULTICORE_API ChCollisionSystemChronoMulticore : public ChCollisionSystemMulticore {
  public:
    ChCollisionSystemChronoMulticore(ChMulticoreDataManager* dc);
    ~ChCollisionSystemChronoMulticore();

    /// Set the number of OpenMP threads for collision detection.
    virtual void SetNumThreads(int nthreads) override;

    /// Synchronization operations, invoked before running the collision detection.
    /// Different from the base class function, this overrides points to already allocated contactable state
    /// information (in the Chrono::Multicore data manager).
    virtual void PreProcess() override;

    /// Synchronization operations, invoked after running the collision detection.
    virtual void PostProcess() override;

    /// Fill in the provided contact container with collision information after Run().
    /// Different from the base class function, this override loads into a ChContactContainerMulticore.
    virtual void ReportContacts(ChContactContainer* container) override;

    /// Fill in the provided proximity container with near point information after Run().
    /// Not used.
    virtual void ReportProximities(ChProximityContainer* mproximitycontainer) override {}

  private:
    ChMulticoreDataManager* data_manager;

    friend class chrono::ChSystemMulticore;
};

/// @} multicore_colision

}  // end namespace chrono

#endif
