// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================

#pragma once

#include "chrono_distributed/ChApiDistributed.h"

#include "chrono_parallel/collision/ChCollisionModelParallel.h"

namespace chrono {

namespace collision {

/// This class adds the ability to track the axis-aligned bounding box for the entire model
/// so that an entire body can be classified by which sub-domains it intersects.
class CH_DISTR_API ChCollisionModelDistributed : public ChCollisionModelParallel {
  public:
    ChCollisionModelDistributed();
    virtual ~ChCollisionModelDistributed();

    /// Delete all inserted geometry.
    virtual int ClearModel() override;

    /// Adds a box collision shape to the model and calculates the model's new AABB
    virtual bool AddBox(double hx, double hy, double hz, const ChVector<>& pos, const ChMatrix33<>& rot) override;

    /// Adds a sphere collision shape to the model and calculates the model's new AABB
    virtual bool AddSphere(double radius, const ChVector<>& pos) override;

    /// Adds a triangle collision shape to the model
    virtual bool AddTriangle(ChVector<> A,
                             ChVector<> B,
                             ChVector<> C,
                             const ChVector<>& pos,
                             const ChMatrix33<>& rot) override;

    /// Gets the axis-aligned bounding box for the entire model
    /// Only valid at beginning of simulation
    virtual void GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const override;

    std::vector<real3> shape_aabb_max;
    std::vector<real3> shape_aabb_min;

  protected:
    /// Upper and lower vertices of the AABB
    ChVector<double> aabb_max;
    ChVector<double> aabb_min;

    /// Indicates that the bounding box has been computed
    bool aabb_valid;
};
}  // namespace collision
}  // namespace chrono
