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
// Authors: Alessandro Tasora, Hammad Mazhar
// =============================================================================
//
// Based on the regular bullet collision system, some modifications made to
// store contacts in the Chrono::Multicore data structures
//
// =============================================================================

#pragma once

#include "chrono/core/ChApiCE.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChProximityContainer.h"

#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/collision/bullet/cbtBulletCollisionCommon.h"
#include "chrono/collision/gimpact/GIMPACT/Bullet/cbtGImpactCollisionAlgorithm.h"

#include "chrono/collision/bullet/LinearMath/cbtPoolAllocator.h"

#include "chrono_multicore/ChMulticoreDefines.h"

namespace chrono {

class ChSystemMulticore;  // forward declaration
class ChMulticoreDataManager;

namespace collision {

/// @addtogroup multicore_collision
/// @{

/// Collision engine based on the 'Bullet' library.
/// Contains both the broadphase and the narrow phase Bullet methods.
class CH_MULTICORE_API ChCollisionSystemBulletMulticore : public ChCollisionSystem {
  public:
    ChCollisionSystemBulletMulticore(ChMulticoreDataManager* dc);
    virtual ~ChCollisionSystemBulletMulticore();

    /// Return the type of this collision system.
    virtual ChCollisionSystemType GetType() const override { return ChCollisionSystemType::BULLET; }

    /// Clear all data instanced by this algorithm if any (like persistent contact manifolds)
    virtual void Clear(void) override;

    /// Add a collision model to the collision engine (custom data may be allocated).
    virtual void Add(ChCollisionModel* model) override;

    /// Remove a collision model from the collision engine.
    virtual void Remove(ChCollisionModel* model) override;

    /// Set the number of OpenMP threads for collision detection.
    virtual void SetNumThreads(int nthreads) override;

    /// Run the algorithm and finds all the contacts.
    /// (Contacts will be managed by the Bullet persistent contact cache).
    virtual void Run() override;

    /// Return an AABB bounding all collision shapes in the system
    virtual void GetBoundingBox(ChVector<>& aabb_min, ChVector<>& aabb_max) const override;

    /// Reset timers for collision detection.
    virtual void ResetTimers() override;

    /// Return the time (in seconds) for broadphase collision detection.
    virtual double GetTimerCollisionBroad() const override;

    /// Return the time (in seconds) for narrowphase collision detection.
    virtual double GetTimerCollisionNarrow() const override;

    /// Fill in the provided contact container with collision information after Run().
    virtual void ReportContacts(ChContactContainer* mcontactcontainer) override;

    /// Fill in the provided proximity container with near point information after Run().
    /// Not used in Chrono::Multicore.
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

    // For Bullet related stuff
    cbtCollisionWorld* GetBulletCollisionWorld() { return bt_collision_world; }

  private:
    cbtCollisionConfiguration* bt_collision_configuration;
    cbtCollisionDispatcher* bt_dispatcher;
    cbtBroadphaseInterface* bt_broadphase;
    cbtCollisionWorld* bt_collision_world;

    ChMulticoreDataManager* data_manager;

    unsigned int counter;
};

/// @} multicore_colision

}  // end namespace collision
}  // end namespace chrono
