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

#ifndef CH_VISUAL_SHAPE_CYLINDER_H
#define CH_VISUAL_SHAPE_CYLINDER_H

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChCylinder.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Class for referencing a cylinder shape that can be visualized in some way.
class ChApi ChVisualShapeCylinder : public ChVisualShape {
  public:
    ChVisualShapeCylinder();
    ChVisualShapeCylinder(double radius, double height);
    ChVisualShapeCylinder(const geometry::ChCylinder& cyl);

    ~ChVisualShapeCylinder(){}

    // Access the cylinder geometry.
    geometry::ChCylinder& GetGeometry() { return gcylinder; }

    /// Get the cylinder radius.
    double GetRadius() const { return gcylinder.GetRadius(); }

    /// Get the cylinder height.
    double GetHeight() const { return gcylinder.GetHeight(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    geometry::ChCylinder gcylinder;
};

/// @} chrono_assets

CH_CLASS_VERSION(ChVisualShapeCylinder, 0)

}  // end namespace chrono

#endif
