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
/// The cone is assumed to be aligned with the z axis of a frame with origin at the cone axis center.
class ChApi ChCone : public ChGeometry {
  public:
    ChCone() : r(0), h(0) {}
    ChCone(double radius, double height) : r(radius), h(height) {}
    ChCone(const ChCone& source);
    ~ChCone() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChCone* Clone() const override { return new ChCone(*this); }

    virtual GeometryType GetClassType() const override { return CONE; }

    /// Compute bounding box along the directions defined by the given rotation matrix.
    /// Note: 'rot' is currently ignored.
    virtual void GetBoundingBox(ChVector<>& cmin, ChVector<>& cmax, const ChMatrix33<>& rot) const override;

    virtual ChVector<> Baricenter() const override { return ChVector<>(); }

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

    double r;
    double h;
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChCone, 0)

}  // end namespace chrono

#endif
