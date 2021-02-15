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

#ifndef CH_COLLISIONSYSTEM_H
#define CH_COLLISIONSYSTEM_H

#include "chrono/collision/ChCollisionInfo.h"
#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"

namespace chrono {

// forward references
class ChBody;
class ChVariablesBody;
class ChContactContainer;
class ChProximityContainer;

/// Namespace with classes for collision detection
namespace collision {

/// Base class for generic collision engine.
class ChApi ChCollisionSystem {
  public:
    ChCollisionSystem() {}
    virtual ~ChCollisionSystem() {}

    /// Clears all data instanced by this algorithm
    /// if any (like persistent contact manifolds)
    virtual void Clear(void) = 0;

    /// Adds a collision model to the collision
    /// engine (custom data may be allocated).
    virtual void Add(ChCollisionModel* model) = 0;

    /// Removes a collision model from the collision
    /// engine (custom data may be deallocated).
    virtual void Remove(ChCollisionModel* model) = 0;

    /// Removes all collision models from the collision
    /// engine (custom data may be deallocated).
    // virtual void RemoveAll() = 0;

    /// Run the collision detection and finds the contacts.
    /// This function will be called at each simulation step.
    virtual void Run() = 0;

    /// Return an AABB bounding all collision shapes in the system
    virtual void GetBoundingBox(ChVector<>& aabb_min, ChVector<>& aabb_max) const = 0;

    /// Return the time (in seconds) for broadphase collision detection.
    virtual double GetTimerCollisionBroad() const = 0;

    /// Return the time (in seconds) for narrowphase collision detection.
    virtual double GetTimerCollisionNarrow() const = 0;

    /// Reset any timers associated with collision detection.
    virtual void ResetTimers() {}

    /// Set the number of OpenMP threads for collision detection.
    /// The default implementation does nothing. Derived classes implement this function as applicable.
    virtual void SetNumThreads(int nthreads) {}

    /// After the Run() has completed, you can call this function to
    /// fill a 'contact container', that is an object inherited from class
    /// ChContactContainer. For instance ChSystem, after each Run()
    /// collision detection, calls this method multiple times for all contact containers in the system,
    /// Children classes _must_ implement this.
    /// The basic behavior of the implementation should be the following: collision system
    /// will call in sequence the functions BeginAddContact(), AddContact() (x n times),
    /// EndAddContact() of the contact container.
    /// In case a specialized implementation (ex. a GPU parallel collision engine)
    /// finds that the contact container is a specialized one (ex with a GPU buffer)
    /// it can call more performant methods to add directly the contacts in batches, for instance
    /// by recognizing that he can call, say, some special AddAllContactsAsGpuBuffer() instead of many AddContact().
    virtual void ReportContacts(ChContactContainer* mcontactcontainer) = 0;

    /// After the Run() has completed, you can call this function to
    /// fill a 'proximity container' (container of narrow phase pairs), that is
    /// an object inherited from class ChProximityContainer. For instance ChSystem, after each Run()
    /// collision detection, calls this method multiple times for all proximity containers in the system,
    /// Children classes _must_ implement this.
    /// The basic behavior of the implementation should be the following: collision system
    /// will call in sequence the functions BeginAddProximities(), AddProximity() (x n times),
    /// EndAddProximities() of the proximity container.
    /// In case a specialized implementation (ex. a GPU parallel collision engine)
    /// finds that the proximity container is a specialized one (ex with a GPU buffer)
    /// it can call more performant methods to add directly the proximities in batches, for instance
    /// by recognizing that he can call, say, some special AddAllProximitiesAsGpuBuffer() instead of many
    /// AddProximity().
    virtual void ReportProximities(ChProximityContainer* mproximitycontainer) = 0;

    /// Class to be used as a callback interface for user-defined actions to be performed
    /// for each 'near enough' pair of collision shapes found by the broad-phase collision step.
    class ChApi BroadphaseCallback {
      public:
        virtual ~BroadphaseCallback() {}

        /// Callback used to process 'near enough' pairs of collision models found by the
        /// broad-phase collision algorithm.
        /// Return false to skip narrow-phase contact generation for this pair of bodies.
        virtual bool OnBroadphase(ChCollisionModel* modelA,  ///< 1st model
                                  ChCollisionModel* modelB   ///< 2nd model
                                  ) = 0;
    };

    /// Specify a callback object to be used each time a pair of 'near enough' collision shapes
    /// is found by the broad-phase collision step. The OnBroadphase() method of the provided
    /// callback object will be called for each pair of 'near enough' shapes.
    void RegisterBroadphaseCallback(std::shared_ptr<BroadphaseCallback> callback) { broad_callback = callback; }

    /// Class to be used as a callback interface for user-defined actions to be performed
    /// at each collision pair found during the narrow-phase collision step.
    /// It can be used to override the geometric information.
    class ChApi NarrowphaseCallback {
      public:
        virtual ~NarrowphaseCallback() {}

        /// Callback used to process collision pairs found by the narrow-phase collision step.
        /// Return true to generate a contact for this pair of overlapping bodies.
        virtual bool OnNarrowphase(ChCollisionInfo& contactinfo) = 0;
    };

    /// Specify a callback object to be used each time a collision pair is found during
    /// the narrow-phase collision detection step. The OnNarrowphase() method of the provided
    /// callback object will be called for each collision pair found during narrow phase.
    void RegisterNarrowphaseCallback(std::shared_ptr<NarrowphaseCallback> callback) { narrow_callback = callback; }

    /// Recover results from RayHit() raycasting.
    struct ChRayhitResult {
        bool hit;                    ///< if true, there was an hit
        ChVector<> abs_hitPoint;     ///< hit point in absolute space coordinates
        ChVector<> abs_hitNormal;    ///< normal to surface in absolute space coordinates
        double dist_factor;          ///< from 0 .. 1 means the distance of hit point along the segment
        ChCollisionModel* hitModel;  ///< pointer to intersected model
    };

    /// Perform a ray-hit test with the collision models.
    virtual bool RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& mresult) const = 0;

    /// Perform a ray-hit test with the specified collision model.
    virtual bool RayHit(const ChVector<>& from,
                        const ChVector<>& to,
                        ChCollisionModel* model,
                        ChRayhitResult& mresult) const = 0;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        // version number
        marchive.VersionWrite<ChCollisionSystem>();
    }

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) {
        // version number
        int version = marchive.VersionRead<ChCollisionSystem>();
    }

  protected:
    std::shared_ptr<BroadphaseCallback> broad_callback;    ///< user callback for each near-enough pair of shapes
    std::shared_ptr<NarrowphaseCallback> narrow_callback;  ///< user callback for each collision pair
};

}  // end namespace collision

CH_CLASS_VERSION(collision::ChCollisionSystem, 0)

}  // end namespace chrono

#endif
