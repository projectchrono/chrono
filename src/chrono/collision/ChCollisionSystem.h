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

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/ChCollisionInfo.h"
#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"
#include "chrono/geometry/ChGeometry.h"
#include "chrono/assets/ChColor.h"

namespace chrono {

// forward references
class ChSystem;
class ChBody;
class ChAssembly;
class ChParticleCloud;
class ChConveyor;
class ChVariablesBody;
class ChContactContainer;
class ChProximityContainer;

/// @addtogroup chrono_collision
/// @{

/// Base class for generic collision engine.
class ChApi ChCollisionSystem {
  public:
    /// Supported collision systems.
    enum class Type {
        BULLET,    ///< Bullet-based collision detection system
        MULTICORE  ///< Chrono multicore collision detection system
    };

    virtual ~ChCollisionSystem();

    /// Test if the collision system was initialized.
    bool IsInitialized() const { return m_initialized; }

    /// Initialize the collision system.
    /// This call must trigger a parsing of the associated Chrono system to process all collision models.
    virtual void Initialize();

    /// Process all collision models in the associated Chrono system.
    /// This function is called by default for a Chrono system attached to this collision system during
    /// initialization, but can also be called later if further modifications to collision models occur.
    virtual void BindAll();

    /// Process the collision models associated with the specified Chrono physics item.
    /// This function must be called if a new physics item is added to the system or if changes to its collision model
    /// occur after the collision system was initialized.
    virtual void BindItem(std::shared_ptr<ChPhysicsItem> item);

    /// Remove any collision models associated with the specified physics item from the collision engine.
    void UnbindItem(std::shared_ptr<ChPhysicsItem> item);

    /// Clears all data instanced by this algorithm if any.
    virtual void Clear() = 0;

    /// Add the specified collision model to the collision engine.
    virtual void Add(std::shared_ptr<ChCollisionModel> model) = 0;

    /// Remove the specified collision model from the collision engine.
    virtual void Remove(std::shared_ptr<ChCollisionModel> model) = 0;

    /// Optional synchronization operations, invoked before running the collision detection.
    virtual void PreProcess() {}

    /// Run the collision detection and finds the contacts.
    /// This function will be called at each simulation step.
    virtual void Run() = 0;

    /// Optional synchronization operations, invoked after running the collision detection.
    virtual void PostProcess() {}

    /// Return an AABB bounding all collision shapes in the system
    virtual ChAABB GetBoundingBox() const = 0;

    /// Return the time (in seconds) for broadphase collision detection.
    virtual double GetTimerCollisionBroad() const = 0;

    /// Return the time (in seconds) for narrowphase collision detection.
    virtual double GetTimerCollisionNarrow() const = 0;

    /// Reset any timers associated with collision detection.
    virtual void ResetTimers() {}

    /// Set the number of OpenMP threads for collision detection.
    /// The default implementation does nothing. Derived classes implement this function as applicable.
    virtual void SetNumThreads(int nthreads) {}

    /// Report contacts (fill the provided 'contact container').
    /// This function, which should only be called after `Run()`, must add to the contact container contacts
    /// corresponding to all detected pairwise collisions. The basic behavior of the implementation should call in
    /// sequence the ChContactContainer functions `BeginAddContact()`, `AddContact()` (multiple times), and
    /// EndAddContact().
    virtual void ReportContacts(ChContactContainer* contact_container) = 0;

    /// Report proximities (fill in the provided 'proximity container').
    /// This function, which should only be called after `Run()`, must add to the contact container contacts
    /// corresponding to all detected pairwise collisions. The basic behavior of the implementation should call in
    /// sequence the ChContactContainer functions `BeginAddProximities()`, `AddProximity()` (multiple times), and
    /// `EndAddProximities()`.
    virtual void ReportProximities(ChProximityContainer* proximity_container) = 0;

    /// Class to be used as a callback interface for user-defined actions to be performed during broadphase.
    /// The `OnBroadphase()` method will be called for each pair of 'near enough' shapes.
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

    /// Register a broadphase callback object.
    void RegisterBroadphaseCallback(std::shared_ptr<BroadphaseCallback> callback) { broad_callback = callback; }

    /// Class to be used as a callback interface for user-defined actions to be performed during narrowphase.
    /// The `OnNarrowphase()` method will be called for each collision pair found during narrow phase.
    class ChApi NarrowphaseCallback {
      public:
        virtual ~NarrowphaseCallback() {}

        /// Callback used to process collision pairs found by the narrow-phase collision step.
        /// Return true to generate a contact for this pair of overlapping bodies.
        virtual bool OnNarrowphase(ChCollisionInfo& cinfo) = 0;
    };

    /// Register a narrowphase callback object.
    void RegisterNarrowphaseCallback(std::shared_ptr<NarrowphaseCallback> callback) { narrow_callback = callback; }

    /// Recover results from RayHit() raycasting.
    struct ChRayhitResult {
        bool hit;                    ///< if true, there was an hit
        ChVector3d abs_hitPoint;     ///< hit point in absolute space coordinates
        ChVector3d abs_hitNormal;    ///< normal to surface in absolute space coordinates
        double dist_factor;          ///< from 0 .. 1 means the distance of hit point along the segment
        ChCollisionModel* hitModel;  ///< pointer to intersected model
    };

    /// Perform a ray-hit test with the collision models.
    virtual bool RayHit(const ChVector3d& from, const ChVector3d& to, ChRayhitResult& result) const = 0;

    /// Perform a ray-hit test with the specified collision model.
    virtual bool RayHit(const ChVector3d& from,
                        const ChVector3d& to,
                        ChCollisionModel* model,
                        ChRayhitResult& result) const = 0;

    /// Class to be used as a callback interface for user-defined visualization of collision shapes.
    class ChApi VisualizationCallback {
      public:
        virtual ~VisualizationCallback() {}

        /// Method for rendering a line of specified color between the two given points.
        virtual void DrawLine(const ChVector3d& from, const ChVector3d& to, const ChColor& color) = 0;

        /// Set scaling factor for normal vectors (default 1.0).
        virtual double GetNormalScale() const { return 1.0; }
    };

    /// Specify a callback object to be used for debug rendering of collision shapes.
    virtual void RegisterVisualizationCallback(std::shared_ptr<VisualizationCallback> callback) {
        vis_callback = callback;
    }

    /// Enumeration of supported flags for collision debug visualization.
    enum VisualizationModes {
        VIS_None = 0,           ///< no debug collision visualization
        VIS_Shapes = 1 << 0,    ///< wireframe representation of collision shapes
        VIS_Aabb = 1 << 1,      ///< axis-aligned bounding boxes of collision shapes
        VIS_Contacts = 1 << 2,  ///< contact points and normals
        VIS_MAX_MODES
    };

    /// Method to trigger debug visualization of collision shapes.
    /// The 'flags' argument can be any of the VisualizationModes enums, or a combination thereof (using bit-wise
    /// operators). The calling program must invoke this function from within the simulation loop. A derived class
    /// should implement a no-op if a visualization callback was not specified.
    virtual void Visualize(int flags) {}

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);

    /// Set associated Chrono system
    void SetSystem(ChSystem* sys) { m_system = sys; }

  protected:
    ChCollisionSystem();

    bool m_initialized;

    ChSystem* m_system;  ///< associated Chrono system

    std::shared_ptr<BroadphaseCallback> broad_callback;    ///< user callback for each near-enough pair of shapes
    std::shared_ptr<NarrowphaseCallback> narrow_callback;  ///< user callback for each collision pair

    std::shared_ptr<VisualizationCallback> vis_callback;  ///< user callback for debug visualization
    int m_vis_flags;
};

/// @} chrono_collision

CH_CLASS_VERSION(ChCollisionSystem, 0)

}  // end namespace chrono

#endif
