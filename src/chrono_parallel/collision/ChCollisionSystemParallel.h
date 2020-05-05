// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: Parallel collsion system that calls a custom AABB generator,
// broadphase and narrowphase
//
// =============================================================================

#pragma once

#include "chrono/physics/ChProximityContainer.h"
#include "chrono/physics/ChBody.h"

#include "chrono/collision/ChCollisionSystem.h"

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/solver/ChSystemDescriptorParallel.h"
#include "chrono_parallel/collision/ChContactContainerParallel.h"
#include "chrono_parallel/collision/ChCollisionModelParallel.h"

namespace chrono {

class ChSystemParallel;  // forward declaration

namespace collision {

/// @addtogroup parallel_collision
/// @{

/// Class for collision engine based on the spatial subdivision method.
/// Contains both the broadphase and the narrow phase methods.
class CH_PARALLEL_API ChCollisionSystemParallel : public ChCollisionSystem {
  public:
    ChCollisionSystemParallel(ChParallelDataManager* dc);
    virtual ~ChCollisionSystemParallel();

    /// Clear all data instanced by this algorithm
    /// if any (like persistent contact manifolds).
    virtual void Clear(void) override {}

    /// Add a collision model to the collision
    /// engine (custom data may be allocated).
    virtual void Add(ChCollisionModel* model) override;

    /// Remove a collision model from the collision engine.
    /// Currently not implemented.
    virtual void Remove(ChCollisionModel* model) override;

    /// Run the algorithm and finds all the contacts.
    virtual void Run() override;

    /// Return the time (in seconds) for broadphase collision detection.
    virtual double GetTimerCollisionBroad() const override;

    /// Return the time (in seconds) for narrowphase collision detection.
    virtual double GetTimerCollisionNarrow() const override;

    /// Fill in the provided contact container with collision information after Run().
    virtual void ReportContacts(ChContactContainer* mcontactcontainer) override;

    /// Fill in the provided proximity container with near point information after Run().
    /// Not used in Chrono::Parallel.
    virtual void ReportProximities(ChProximityContainer* mproximitycontainer) override {}

    /// Perform a ray-hit test with all collision models.
    /// Currently not implemented.
    virtual bool RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& mresult) const override {
        return false;
    }

    /// Perform a ray-hit test with the specified collision model.
    /// Currently not implemented.
    virtual bool RayHit(const ChVector<>& from,
                        const ChVector<>& to,
                        ChCollisionModel* model,
                        ChRayhitResult& mresult) const override {
        return false;
    }


    /// Set and enable "active" box.
    /// Bodies outside this AABB are deactivated.
    void SetAABB(real3 aabbmin, real3 aabbmax);

    /// Get the dimensions of the "active" box.
    /// The return value indicates whether or not the active box feature is enabled.
    bool GetAABB(real3& aabbmin, real3& aabbmax);

    /// Mark bodies whose AABB is contained within the specified box.
    virtual void GetOverlappingAABB(custom_vector<char>& active_id, real3 Amin, real3 Amax);

    /// Return the pairs of IDs for overlapping contact shapes.
    virtual std::vector<vec2> GetOverlappingPairs();

  private:
    ChParallelDataManager* data_manager;
    custom_vector<char> body_active;

    friend class ChSystemParallel;
};

/// @} parallel_colision

}  // end namespace collision
}  // end namespace chrono
