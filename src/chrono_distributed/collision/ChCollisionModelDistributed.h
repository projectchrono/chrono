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

#include "chrono/collision/ChCollisionModelChrono.h"

namespace chrono {
namespace collision {

/// @addtogroup distributed_collision
/// @{

/// This class adds the ability to track the axis-aligned bounding box for the entire model
/// so that an entire body can be classified by which sub-domains it intersects.
class CH_DISTR_API ChCollisionModelDistributed : public ChCollisionModelChrono {
  public:
    ChCollisionModelDistributed();
    virtual ~ChCollisionModelDistributed();

    /// Delete all inserted geometry.
    virtual int ClearModel() override;

    /// Adds a box collision shape to the model and calculates the model's new AABB
    virtual bool AddBox(std::shared_ptr<ChMaterialSurface> material,  ///< Contact material
                        double size_x,                                ///< dimension in X direction
                        double size_y,                                ///< dimension in Y direction
                        double size_z,                                ///< dimension in Z direction
                        const ChVector<>& pos = ChVector<>(),         ///< Box center location
                        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< Box orientation
                        ) override;

    /// Adds a sphere collision shape to the model and calculates the model's new AABB
    virtual bool AddSphere(std::shared_ptr<ChMaterialSurface> material,  ///< Contact material
                           double radius,                                ///< Sphere radius
                           const ChVector<>& pos = ChVector<>()          ///< Sphere center location
                           ) override;

    /// Adds a triangle collision shape to the model
    virtual bool AddTriangle(std::shared_ptr<ChMaterialSurface> material,  ///< Contact material
                             ChVector<> A,                                 ///< Vertex of triangle
                             ChVector<> B,                                 ///< Vertex of triangle
                             ChVector<> C,                                 ///< Vertex of triangle
                             const ChVector<>& pos = ChVector<>(),         ///< Triangle position
                             const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< Triangle orientation
                             ) override;

    /// Gets the axis-aligned bounding box for the entire model
    /// Only valid at beginning of simulation
    virtual void GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const override;

    std::vector<real3> shape_aabb_max;  ///< Upper AABB corners for each shape in model
    std::vector<real3> shape_aabb_min;  ///< Lower AABB corners for each shape in model

  protected:
    ChVector<double> aabb_max;  ///< Upper corner of model AABB
    ChVector<double> aabb_min;  ///< Lower corner of model AABB

    bool aabb_valid;  ///< Indicates that the bounding box has been computed
};

/// @} distributed_collision

}  // namespace collision
}  // namespace chrono
