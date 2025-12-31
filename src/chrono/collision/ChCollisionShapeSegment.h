// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_COLLISION_SHAPE_SEGMENT_H
#define CH_COLLISION_SHAPE_SEGMENT_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/core/ChVector3.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Collision segment shape.
class ChApi ChCollisionShapeSegment : public ChCollisionShape {
  public:
    ChCollisionShapeSegment();
    ChCollisionShapeSegment(std::shared_ptr<ChContactMaterial> material,
                            const ChVector3d* point1,
                            const ChVector3d* point2,
                            bool owns_point1,
                            bool owns_point2,
                            double radius);

    ~ChCollisionShapeSegment() {}

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    const ChVector3d* P1;
    const ChVector3d* P2;
    bool ownsP1;
    bool ownsP2;
    double radius;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
