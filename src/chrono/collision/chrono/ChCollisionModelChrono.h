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

#ifndef CH_COLLISION_MODEL_CHRONO_H
#define CH_COLLISION_MODEL_CHRONO_H

#include "chrono/collision/ChCollisionModel.h"

#include "chrono/multicore_math/ChMulticoreMath.h"

namespace chrono {

// Forward references
class ChBody;

namespace collision {

/// @addtogroup collision_mc
/// @{

/// Geometric model for the custom multicore Chrono collision system.
class ChApi ChCollisionModelChrono : public ChCollisionModel {
  public:
    ChCollisionModelChrono();
    virtual ~ChCollisionModelChrono();

    /// Return the type of this collision model.
    virtual ChCollisionSystemType GetType() const override { return ChCollisionSystemType::CHRONO; }

    /// Add all shapes already contained in another model.
    virtual bool AddCopyOfAnotherModel(ChCollisionModel* another) override;

    /// Sets the position and orientation of the collision
    /// model as the rigid body current position.
    virtual void SyncPosition() override;

    /// Return a pointer to the associated body.
    ChBody* GetBody() const { return mbody; }

    /// Set the pointer to the owner rigid body.
    void SetBody(ChBody* body) { mbody = body; }

    /// Sets the pointer to the contactable object.
    virtual void SetContactable(ChContactable* mc) override;

    /// Return the axis aligned bounding box for this collision model.
    virtual void GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const override;

    /// Return the position and orientation of the collision shape with specified index, relative to the model frame.
    virtual ChCoordsys<> GetShapePos(int index) const override;

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

    /// Remove this model from the collision system (if applicable).
    virtual void Dissociate() override;

    /// Insert this model into the collision system (if applicable).
    virtual void Associate() override;

    /// Populate the collision system with the collision shapes defined in this model.
    virtual void Populate() override;

    ChBody* mbody;                                               ///< associated contactable (rigid body only)
    std::vector<std::shared_ptr<ctCollisionShape>> m_ct_shapes;  ///< list of Chrono collision shapes in model

    friend class ChCollisionSystemChrono;
};

/// @} collision_mc

}  // end namespace collision
}  // end namespace chrono

#endif
