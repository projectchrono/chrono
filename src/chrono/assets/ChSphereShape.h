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

#ifndef CHOBJSPHERESHAPE_H
#define CHOBJSPHERESHAPE_H

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChSphere.h"

namespace chrono {

/// A spherical geometric object for collisions and visualization.
class ChApi ChSphereShape : public ChVisualShape {
  public:
    ChSphereShape();
    ChSphereShape(double radius);
    ChSphereShape(const geometry::ChSphere& sphere);

    ~ChSphereShape() {}

    /// Access the sphere geometry.
    geometry::ChSphere& GetGeometry() { return gsphere; }

    /// Get the sphere radius.
    double GetRadius() const { return gsphere.GetRadius(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    geometry::ChSphere gsphere;
};

CH_CLASS_VERSION(ChSphereShape, 0)

}  // end namespace chrono

#endif
