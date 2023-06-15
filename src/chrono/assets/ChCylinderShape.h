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

#ifndef CHCYLINDERSHAPE_H
#define CHCYLINDERSHAPE_H

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChCylinder.h"

namespace chrono {

/// Class for referencing a cylinder shape that can be visualized in some way.
class ChApi ChCylinderShape : public ChVisualShape {
  public:
    ChCylinderShape();
    ChCylinderShape(double radius, double height);
    ChCylinderShape(const geometry::ChCylinder& cyl);

    ~ChCylinderShape(){}

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

CH_CLASS_VERSION(ChCylinderShape, 0)

}  // end namespace chrono

#endif
