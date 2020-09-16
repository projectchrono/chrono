// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CH_COLLISION_SYSTEM_BULLET_H
#define CH_COLLISION_SYSTEM_BULLET_H

#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/collision/bullet/btBulletCollisionCommon.h"
#include "chrono/core/ChApiCE.h"

namespace chrono {
namespace collision {


/// Collision engine based on the 'Bullet' library.
/// Contains both the broadphase and the narrow phase Bullet methods.
class ChApi ChCollisionSystemBullet : public ChCollisionSystem {
  public:
    ChCollisionSystemBullet();
    virtual ~ChCollisionSystemBullet();

    /// Clears all data instanced by this algorithm
    /// if any (like persistent contact manifolds)
    virtual void Clear(void) override;

    /// Adds a collision model to the collision
    /// engine (custom data may be allocated).
    virtual void Add(ChCollisionModel* model) override;

    /// Removes a collision model from the collision
    /// engine (custom data may be deallocated).
    virtual void Remove(ChCollisionModel* model) override;

    /// Removes all collision models from the collision
    /// engine (custom data may be deallocated).
    // virtual void RemoveAll();

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

    /// After the Run() has completed, you can call this function to
    /// fill a 'contact container', that is an object inherited from class
    /// ChContactContainer. For instance ChSystem, after each Run()
    /// collision detection, calls this method multiple times for all contact containers in the system,
    /// The basic behavior of the implementation is the following: collision system
    /// will call in sequence the functions BeginAddContact(), AddContact() (x n times),
    /// EndAddContact() of the contact container.
    virtual void ReportContacts(ChContactContainer* mcontactcontainer) override;

    /// After the Run() has completed, you can call this function to
    /// fill a 'proximity container' (container of narrow phase pairs), that is
    /// an object inherited from class ChProximityContainer. For instance ChSystem, after each Run()
    /// collision detection, calls this method multiple times for all proximity containers in the system,
    /// The basic behavior of the implementation is  the following: collision system
    /// will call in sequence the functions BeginAddProximities(), AddProximity() (x n times),
    /// EndAddProximities() of the proximity container.
    virtual void ReportProximities(ChProximityContainer* mproximitycontainer) override;

    /// Perform a ray-hit test with all collision models.
    virtual bool RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& mresult) const override;

    /// Perform a ray-hit test with all collision models. This version allows specifying the Bullet
    /// collision filter group and mask (see btBroadphaseProxy::CollisionFilterGroups).
    bool RayHit(const ChVector<>& from,
                const ChVector<>& to,
                ChRayhitResult& mresult,
                short int filter_group,
                short int filter_mask) const;

    /// Perform a ray-hit test with the specified collision model.
    virtual bool RayHit(const ChVector<>& from,
                        const ChVector<>& to,
                        ChCollisionModel* model,
                        ChRayhitResult& mresult) const override;

    /// Perform a ray-hit test with the specified collision model. This version allows specifying the Bullet
    /// collision filter group and mask (see btBroadphaseProxy::CollisionFilterGroups).
    bool RayHit(const ChVector<>& from,
                const ChVector<>& to,
                ChCollisionModel* model,
                ChRayhitResult& mresult,
                short int filter_group,
                short int filter_mask) const;

    // For Bullet related stuff
    btCollisionWorld* GetBulletCollisionWorld() { return bt_collision_world; }

    // Tweak the default contact breaking/merging threshold tolerance
    // of Bullet (is it the static gContactBreakingThreshold scalar in Bullet).
    // Call it only once, before running the simulation.
    static void SetContactBreakingThreshold(double threshold);

  private:
    btCollisionConfiguration* bt_collision_configuration;
    btCollisionDispatcher* bt_dispatcher;
    btBroadphaseInterface* bt_broadphase;
    btCollisionWorld* bt_collision_world;

    btCollisionAlgorithmCreateFunc* m_collision_arc_seg;
    btCollisionAlgorithmCreateFunc* m_collision_seg_arc;
    btCollisionAlgorithmCreateFunc* m_collision_arc_arc;
    btCollisionAlgorithmCreateFunc* m_collision_cetri_cetri;
    void* m_tmp_mem;
    btCollisionAlgorithmCreateFunc* m_emptyCreateFunc;
};

}  // end namespace collision
}  // end namespace chrono

#endif
