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

#ifndef CHC_CONE_H
#define CHC_CONE_H

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

/// A conical geometric object for collisions and visualization.
class ChApi ChCone : public ChGeometry {
  public:
    ChCone() : h(0), r(0) {}
    ChCone(double radius, double height) : r(radius), h(height) {}
    ChCone(const ChCone& source);
    ~ChCone() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChCone* Clone() const override { return new ChCone(*this); }

    /// Get the class type as an enum.
    virtual Type GetClassType() const override { return Type::CONE; }

    /// Compute bounding box along the directions defined by the given rotation matrix.
    /// Note: 'rot' is currently ignored.
    virtual AABB GetBoundingBox(const ChMatrix33<>& rot) const override;

    /// Compute the baricenter of the cone.
    virtual ChVector<> Baricenter() const override { return ChVector<>(); }

    /// Get the cone radius.
    double GetRadius() const { return r; }

    /// Get the cone height.
    double GetHeight() const { return h; }

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    double h;
    double r;
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChCone, 0)

}  // end namespace chrono

#endif
