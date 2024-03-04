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
// Authors: Radu Serban, Hammad Mazhar
// =============================================================================
// Geometric model for the custom multicore Chrono collision system
// =============================================================================

#ifndef CH_COLLISION_MODEL_MULTICORE_H
#define CH_COLLISION_MODEL_MULTICORE_H

#include "chrono/collision/ChCollisionModel.h"

#include "chrono/multicore_math/ChMulticoreMath.h"

namespace chrono {

// Forward references
class ChBody;

/// @addtogroup collision_mc
/// @{

/// Geometric model for the custom multicore Chrono collision system.
class ChApi ChCollisionModelMulticore : public ChCollisionModelImpl {
  public:
    ChCollisionModelMulticore(ChCollisionModel* collision_model);
    virtual ~ChCollisionModelMulticore();

    /// Sets the position and orientation of the collision
    /// model as the rigid body current position.
    virtual void SyncPosition() override;

    /// Return a pointer to the associated body.
    ChBody* GetBody() const { return mbody; }

    /// Set the pointer to the owner rigid body.
    void SetBody(ChBody* body) { mbody = body; }

    /// Return the axis aligned bounding box for this collision model.
    virtual geometry::ChAABB GetBoundingBox() const override;

    std::vector<real3> local_convex_data;

    ChVector<> aabb_min;
    ChVector<> aabb_max;

  protected:
    struct ctCollisionShape {
        ctCollisionShape() : convex(nullptr) {}

        real3 A;        // location
        real3 B;        // dimensions
        real3 C;        // extra
        quaternion R;   // rotation
        real3* convex;  // pointer to convex data;

        real3 aabb_min;  // lower corner of shape AABB
        real3 aabb_max;  // upper corner of shape AABB
    };

    /// Populate the collision system with the collision shapes defined in this model.
    void Populate();

    /// Additional operations to be performed on a change in collision family.
    virtual void OnFamilyChange(short int family_group, short int family_mask) override {}

    ChBody* mbody;                                               ///< associated contactable (rigid body only)
    std::vector<std::shared_ptr<ctCollisionShape>> m_ct_shapes;  ///< list of Chrono collision shapes in model
    std::vector<std::shared_ptr<ChCollisionShape>> m_shapes;     ///< extended list of collision shapes

    friend class ChCollisionSystemMulticore;
    friend class ChCollisionSystemChronoMulticore;
    friend class ChContactContainerMulticoreNSC;
    friend class ChContactContainerMulticoreSMC;
    
};

/// @} collision_mc

}  // end namespace chrono

#endif
