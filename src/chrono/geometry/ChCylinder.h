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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHC_CYLINDER_H
#define CHC_CYLINDER_H

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

/// A cylindrical geometric object for collisions and visualization.
class ChApi ChCylinder : public ChGeometry {
  public:
    ChCylinder() : r(0), h(0) {}
    ChCylinder(double radius, double height) : r(radius), h(height) {}
    ChCylinder(const ChCylinder& source);
    ~ChCylinder() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChCylinder* Clone() const override { return new ChCylinder(*this); }

    /// Get the class type as an enum.
    virtual Type GetClassType() const override { return Type::CYLINDER; }

    /// Compute bounding box along the directions defined by the given rotation matrix.
    virtual AABB GetBoundingBox(const ChMatrix33<>& rot) const override;

    /// Compute the baricenter of the capsule.
    virtual ChVector<> Baricenter() const override { return ChVector<>(0); }

    /// Get the cylinder radius.
    double GetRadius() const { return r; }

    /// Get the cylinder height.
    double GetHeight() const { return h; }

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    /// Utility function for calculating the length and frame of a segment between two given points.
    /// The resulting frame is centered at the midpoint and has the Z axis along the segment.

    double r;  ///< cylinder radius
    double h;  ///< cylinder height
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChCylinder, 0)

}  // end namespace chrono

#endif
