//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_COLLISIONSYSTEM_H
#define CHC_COLLISIONSYSTEM_H

#include "chrono/collision/ChCCollisionInfo.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChApiCE.h"

namespace chrono {

// forward references
class ChBody;
class ChVariablesBody;
class ChContactContainerBase;
class ChProximityContainerBase;

/// Namespace with classes for collision detection
namespace collision {

///
/// Class to be used as a callback interface for the narrow phase collision
/// system. For each contact point found during the Run() execution
/// of the ChCollisionSystem, the function ChNarrowPhaseCallback() is called.
/// The user should implement an inherited class and
/// implement a custom NarrowCallback() function.
///

class ChApi ChNarrowPhaseCallback {
  public:
    /// Callback, used to report contact points found by default
    /// narrow phase collision step.
    /// This must be implemented by a child class of ChNarrowPhaseCallback
    virtual void NarrowCallback(ChCollisionInfo& mcontactinfo) = 0;
};

///
/// Class to be used as a callback interface for the broad-phase collision
/// system. For each 'near enough' pair of shapes found during the Run() execution
/// of the ChCollisionSystem, the function ChBroadPhaseCallback() is called.
/// The user could optionally implement an inherited class and
/// implement a custom BroadCallback().
///

class ChApi ChBroadPhaseCallback {
  public:
    /// Callback used to report 'near enough' pairs of models.
    /// This must be implemented by a child class of ChBroadPhaseCallback.
    /// Return false to skip narrow-phase contact generation for this pair of bodies.
    virtual bool BroadCallback(ChCollisionModel* mmodelA,  ///< pass 1st model
                               ChCollisionModel* mmodelB   ///< pass 2nd model
                               ) = 0;
};

///
/// Base class for generic collision engine.
/// Most methods are 'pure virtual': they need to be implemented
/// by child classes.
///

class ChApi ChCollisionSystem {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChCollisionSystem)

  public:
    ChCollisionSystem(unsigned int max_objects = 16000, double scene_size = 500) {
        narrow_callback = 0;
        broad_callback = 0;
    };

    virtual ~ChCollisionSystem(){};

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

    /// RUN THE ALGORITHM and finds the contacts.
    /// This is the most important function - it will be called
    /// at each simulation step.
    /// Children classes _must_ implement this.
    virtual void Run() = 0;

    /// After the Run() has completed, you can call this function to
    /// fill a 'contact container', that is an object inherited from class
    /// ChContactContainerBase. For instance ChSystem, after each Run()
    /// collision detection, calls this method multiple times for all contact containers in the system,
    /// Children classes _must_ implement this.
    /// The basic behavior of the implementation should be the following: collision system
    /// will call in sequence the functions BeginAddContact(), AddContact() (x n times),
    /// EndAddContact() of the contact container.
    /// In case a specialized implementation (ex. a GPU parallel collision engine)
    /// finds that the contact container is a specialized one (ex with a GPU buffer)
    /// it can call more performant methods to add directly the contacts in batches, for instance
    /// by recognizing that he can call, say, some special AddAllContactsAsGpuBuffer() instead of many AddContact().
    virtual void ReportContacts(ChContactContainerBase* mcontactcontainer) = 0;

    /// After the Run() has completed, you can call this function to
    /// fill a 'proximity container' (container of narrow phase pairs), that is
    /// an object inherited from class ChProximityContainerBase. For instance ChSystem, after each Run()
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
    virtual void ReportProximities(ChProximityContainerBase* mproximitycontainer) = 0;

    /// Sets the user ChBroadPhaseCallback to be used
    /// to tell the near-enough pairs found during the Run()
    /// execution. It will be executed for near enough pair of bodies.
    void SetBroadPhaseCallback(ChBroadPhaseCallback* mcallback) { broad_callback = mcallback; }

    /// Sets the used ChNarrowPhaseCallback to be used
    /// to report the contacts found during the Run()
    /// execution. It will be executed for each contact point.
    void SetNarrowPhaseCallback(ChNarrowPhaseCallback* mcallback) { narrow_callback = mcallback; }

    /// This will be used to recover results from RayHit() raycasting
    struct ChRayhitResult {
        bool hit;                    /// if true, there was an hit - look following date for infos
        ChVector<> abs_hitPoint;     /// hit point in absolute space coordinates
        ChVector<> abs_hitNormal;    /// normal to surface in absolute space coordinates
        double dist_factor;          /// from 0 .. 1 means the distance of hit point along the segment
        ChCollisionModel* hitModel;  /// pointer to hitten model
    };
    /// Perform a ray-hit test with the collision models.
    virtual bool RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& mresult) = 0;

    // SERIALIZATION

    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        // version number
        marchive.VersionWrite<ChCollisionSystem>();
    }
    virtual void ArchiveIN(ChArchiveIn& marchive) {
        // version number
        int version = marchive.VersionRead<ChCollisionSystem>();
    }

  protected:
    ChBroadPhaseCallback* broad_callback;    // user callback for each near-enough pair of shapes
    ChNarrowPhaseCallback* narrow_callback;  // user callback for each contact
};


}  // end namespace collision

CH_CLASS_VERSION(collision::ChCollisionSystem,0)

}  // end namespace chrono

#endif
