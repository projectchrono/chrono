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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHVISUAL_SHAPE_CAPSULE_H
#define CHVISUAL_SHAPE_CAPSULE_H

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChCapsule.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Class for referencing a capsule shape that can be visualized in some way.
class ChApi ChVisualShapeCapsule : public ChVisualShape {
  public:
    ChVisualShapeCapsule();
    ChVisualShapeCapsule(double radius, double height);
    ChVisualShapeCapsule(const geometry::ChCapsule& cap);

    ~ChVisualShapeCapsule() {}

    /// Access the capsule geometry.
    geometry::ChCapsule& GetGeometry() { return gcapsule; }

    /// Get the capsule radius.
    double GetRadius() const { return gcapsule.GetRadius(); }

    /// Get the capsule height (length of cylindrical portion).
    double GetHeight() const { return gcapsule.GetHeight(); }

    /// Get the capsule total length.
    double GetLength() const { return gcapsule.GetLength(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    geometry::ChCapsule gcapsule;
};

/// @} chrono_assets

CH_CLASS_VERSION(ChVisualShapeCapsule, 0)

}  // end namespace chrono

#endif
