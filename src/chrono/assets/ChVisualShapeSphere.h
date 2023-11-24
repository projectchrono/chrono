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

#ifndef CH_VISUAL_SHAPE_SPHERE_H
#define CH_VISUAL_SHAPE_SPHERE_H

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChSphere.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// A spherical geometric object for collisions and visualization.
class ChApi ChVisualShapeSphere : public ChVisualShape {
  public:
    ChVisualShapeSphere();
    ChVisualShapeSphere(double radius);
    ChVisualShapeSphere(const geometry::ChSphere& sphere);

    ~ChVisualShapeSphere() {}

    /// Access the sphere geometry.
    geometry::ChSphere& GetGeometry() { return gsphere; }

    /// Get the sphere radius.
    double GetRadius() const { return gsphere.GetRadius(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    geometry::ChSphere gsphere;
};

/// @} chrono_assets

CH_CLASS_VERSION(ChVisualShapeSphere, 0)

}  // end namespace chrono

#endif
