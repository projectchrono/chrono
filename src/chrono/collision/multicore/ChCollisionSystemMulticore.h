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
// Chrono custom multicore collision system.
// Contains both the broadphase and the narrow phase methods.
//
// =============================================================================

#ifndef CH_COLLISION_SYSTEM_MULTICORE_H
#define CH_COLLISION_SYSTEM_MULTICORE_H

#include "chrono/core/ChTimer.h"

#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/collision/multicore/ChCollisionModelMulticore.h"
#include "chrono/collision/multicore/ChCollisionData.h"
#include "chrono/collision/multicore/ChBroadphase.h"
#include "chrono/collision/multicore/ChNarrowphase.h"

namespace chrono {

// forward references
class ChAssembly;
class ChParticleCloud;

/// @addtogroup collision_mc
/// @{

/// Chrono multicore collision system.
/// Contains both the broadphase and the narrow phase methods.
class ChApi ChCollisionSystemMulticore : public ChCollisionSystem {
  public:
    ChCollisionSystemMulticore();
    virtual ~ChCollisionSystemMulticore();

    /// Clears all data instanced by this algorithm
    /// if any (like persistent contact manifolds)
    virtual void Clear() override;

    /// Add the specified collision model to the collision engine.
    virtual void Add(std::shared_ptr<ChCollisionModel> model) override;

    /// Remove the specified collision model from the collision engine.
    virtual void Remove(std::shared_ptr<ChCollisionModel> model) override;

    /// Set collision envelope for rigid shapes (default: ChCollisionModel::GetDefaultSuggestedEnvelope).
    /// For stability of NSC contact, the envelope should be set to 5-10% of the smallest collision shape size (too
    /// large a value will slow down the narrowphase collision detection). The envelope is the amount by which each
    /// collision shape is inflated prior to performing the collision detection, in order to create contact constraints
    /// before shapes actually come in contact. This collision detection system uses a global envelope, used for all
    /// rigid shapes in the system.
    void SetEnvelope(double envelope);

    /// Set a fixed number of grid bins (default 10x10x10).
    /// This is the default setting; to continuously adjust the number of bins, use SetBroadphaseGridSize or
    /// SetBroadphaseGridDensity.
    void SetBroadphaseGridResolution(const ChVector3i& num_bins);

    /// Set a variable number of grids, such that each bin has roughly the specified size.
    /// By default, a fixed grid resolution is used (see SetBroadphaseGridResolution).
    void SetBroadphaseGridSize(const ChVector3d& bin_size);

    /// Set a variable number of grid bins, such that there are roughly `density` collision shapes per bin.
    /// By default, a fixed number of bins is used (see SetBroadphaseGridResolution).
    void SetBroadphaseGridDensity(double density);

    /// Set the narrowphase algorithm (default: ChNarrowphase::Algorithm::HYBRID).
    /// The Chrono collision detection system provides several analytical collision detection algorithms, for particular
    /// pairs of shapes (see ChNarrowphasePRIMS). For general convex shapes, the collision system relies on the
    /// Minkovski Portal Refinement algorithm (see ChNarrowphaseMPR).
    void SetNarrowphaseAlgorithm(ChNarrowphase::Algorithm algorithm);

    /// Enable monitoring of shapes outside active bounding box (default: false).
    /// If enabled, objects whose collision shapes exit the active bounding box are deactivated (frozen).
    /// The size of the bounding box is specified by its min and max extents.
    void EnableActiveBoundingBox(const ChVector3d& aabb_min, const ChVector3d& aabb_max);

    /// Get the dimensions of the "active" box.
    /// The return value indicates whether or not the active box feature is enabled.
    bool GetActiveBoundingBox(ChVector3d& aabb_min, ChVector3d& aabb_max) const;

    /// Set the number of OpenMP threads for collision detection.
    virtual void SetNumThreads(int nthreads) override;

    /// Synchronization operations, invoked before running the collision detection.
    /// This function copies contactable state information in the collision system's data structures.
    virtual void PreProcess() override;

    /// Run the algorithm and finds all the contacts.
    virtual void Run() override;

    /// Synchronization operations, invoked after running the collision detection.
    /// This function updates the list of active bodies (if active bounding box enabled).
    virtual void PostProcess() override;

    /// Return an AABB bounding all collision shapes in the system.
    virtual ChAABB GetBoundingBox() const override;

    /// Reset any timers associated with collision detection.
    virtual void ResetTimers() override;

    /// Return the time (in seconds) for broadphase collision detection.
    virtual double GetTimerCollisionBroad() const override;

    /// Return the time (in seconds) for narrowphase collision detection.
    virtual double GetTimerCollisionNarrow() const override;

    /// Fill in the provided contact container with collision information after Run().
    virtual void ReportContacts(ChContactContainer* container) override;

    /// Fill in the provided proximity container with near point information after Run().
    /// Not used.
    virtual void ReportProximities(ChProximityContainer* mproximitycontainer) override {}

    /// Perform a ray-hit test with all collision models.
    /// Currently not implemented.
    virtual bool RayHit(const ChVector3d& from, const ChVector3d& to, ChRayhitResult& result) const override;

    /// Perform a ray-hit test with the specified collision model.
    /// Currently not implemented.
    virtual bool RayHit(const ChVector3d& from,
                        const ChVector3d& to,
                        ChCollisionModel* model,
                        ChRayhitResult& result) const override;

    /// Method to trigger debug visualization of collision shapes.
    /// The 'flags' argument can be any of the VisualizationModes enums, or a combination thereof (using bit-wise
    /// operators). The calling program must invoke this function from within the simulation loop. No-op if a
    /// visualization callback was not specified with RegisterVisualizationCallback().
    virtual void Visualize(int flags) override;

    /// Return the pairs of IDs for overlapping contact shapes.
    virtual std::vector<vec2> GetOverlappingPairs();

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    /// Mark bodies whose AABB is contained within the specified box.
    virtual void GetOverlappingAABB(std::vector<char>& active_id, real3 Amin, real3 Amax);

    /// Generate the current axis-aligned bounding boxes of collision shapes.
    void GenerateAABB();

    /// Visualize collision shapes (wireframe).
    void VisualizeShapes();

    ///  Visualize collision shape AABBs
    void VisualizeAABB();

    /// Visualize contact points and normals.
    void VisualizeContacts();

    std::vector<std::shared_ptr<ChCollisionModelMulticore>> ct_models;

    std::shared_ptr<ChCollisionData> cd_data;

    ChBroadphase broadphase;    ///< methods for broad-phase collision detection
    ChNarrowphase narrowphase;  ///< methods for narrow-phase collision detection

    std::vector<char> body_active;

    bool use_aabb_active;   ///< enable freezing of objects outside the active bounding box
    real3 active_aabb_min;  ///< lower corner of active bounding box
    real3 active_aabb_max;  ///< upper corner of active bounding box

    ChTimer m_timer_broad;
    ChTimer m_timer_narrow;
};

/// @} collision_mc

}  // end namespace chrono

#endif
