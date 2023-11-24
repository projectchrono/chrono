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

#ifndef CHROUNDEDCYLINDERSHAPE_H
#define CHROUNDEDCYLINDERSHAPE_H

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChRoundedCylinder.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Class for referencing a rounded cylinder shape that can be visualized in some way.
class ChApi ChVisualShapeRoundedCylinder : public ChVisualShape {
  public:
    ChVisualShapeRoundedCylinder();
    ChVisualShapeRoundedCylinder(double radius, double height, double sphere_radius);
    ChVisualShapeRoundedCylinder(const geometry::ChRoundedCylinder& cyl);

    ~ChVisualShapeRoundedCylinder() {}

    // Access the rounded cylinder geometry.
    geometry::ChRoundedCylinder& GetGeometry() { return groundedcyl; }

    /// Get the cylinder radius.
    double GetRadius() const { return groundedcyl.GetRadius(); }

    /// Get the cylinder height.
    double GetHeight() const { return groundedcyl.GetHeight(); }

    /// Get the radius of the sweeping sphere.
    double GetSphereRadius() const { return groundedcyl.GetSphereRadius(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    geometry::ChRoundedCylinder groundedcyl;
};

/// @} chrono_assets

CH_CLASS_VERSION(ChVisualShapeRoundedCylinder, 0)

}  // end namespace chrono

#endif
