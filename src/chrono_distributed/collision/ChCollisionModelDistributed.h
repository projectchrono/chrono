// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#pragma once

#include "chrono_distributed/ChApiDistributed.h"

#include "chrono/collision/chrono/ChCollisionModelChrono.h"

namespace chrono {
namespace collision {

/// @addtogroup distributed_collision
/// @{

/// Specialization of ChCollisionModelChrono to track the model AABB.
/// The model AABB is used in testing intersections with sub-domains. 
class CH_DISTR_API ChCollisionModelDistributed : public ChCollisionModelChrono {
  public:
    ChCollisionModelDistributed();
    ~ChCollisionModelDistributed();

    /// Return the current axis aligned bounding box (AABB) of the collision model.
    /// Only valid at beginning of simulation.
    virtual void GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const override;

  private:
    /// Remove this model from the collision system (if applicable).
    virtual void Dissociate() override;

    /// Insert this model into the collision system (if applicable).
    virtual void Associate() override;

    /// Populate the collision system with the collision shapes defined in this model.
    virtual void Populate() override;

    std::shared_ptr<ctCollisionShape> GetCtShape(int index) const { return m_ct_shapes[index]; }

    real3 aabb_max;  ///< upper corner of model AABB
    real3 aabb_min;  ///< lower corner of model AABB

    bool aabb_valid;  ///< true if model AABB was initialized 

    friend class ChCollisionBoundaryDistributed;
    friend class ChCollisionSystemDistributed;
};

/// @} distributed_collision

}  // namespace collision
}  // namespace chrono
