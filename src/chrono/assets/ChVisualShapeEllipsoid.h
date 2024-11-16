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

#ifndef CH_VISUAL_SHAPE_ELLIPSOID_H
#define CH_VISUAL_SHAPE_ELLIPSOID_H

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChEllipsoid.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Class for referencing an ellipsoid shape that can be visualized in some way.
class ChApi ChVisualShapeEllipsoid : public ChVisualShape {
  public:
    ChVisualShapeEllipsoid();
    ChVisualShapeEllipsoid(double axis_x, double axis_y, double axis_z);
    ChVisualShapeEllipsoid(const ChVector3d& axes);
    ChVisualShapeEllipsoid(const ChEllipsoid& ellipsoid);

    ~ChVisualShapeEllipsoid(){};

    // Access the ellipsoid geometry.
    ChEllipsoid& GetGeometry() { return gellipsoid; }

    /// Get the ellipsoid semiaxes.
    const ChVector3d& GetSemiaxes() const { return gellipsoid.GetSemiaxes(); }

    /// Get the ellipsoid axes.
    ChVector3d GetAxes() const { return gellipsoid.GetAxes(); }

    /// Get the shape bounding box.
    virtual ChAABB GetBoundingBox() const override { return gellipsoid.GetBoundingBox(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    ChEllipsoid gellipsoid;
};

/// @} chrono_assets

CH_CLASS_VERSION(ChVisualShapeEllipsoid, 0)

}  // end namespace chrono

#endif
