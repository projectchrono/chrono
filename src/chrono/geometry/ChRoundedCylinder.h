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

#ifndef CHC_ROUNDEDCYLINDER_H
#define CHC_ROUNDEDCYLINDER_H

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

/// A rounded cylinder (sphere-swept cylinder) geometric object for collision and visualization.
class ChApi ChRoundedCylinder : public ChGeometry {
  public:
    ChRoundedCylinder() : r(0), h(0), sr(0) {}
    ChRoundedCylinder(double radius, double height, double sphere_radius);
    ChRoundedCylinder(const ChRoundedCylinder& source);
    ~ChRoundedCylinder() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChRoundedCylinder* Clone() const override { return new ChRoundedCylinder(*this); }

    virtual Type GetClassType() const override { return Type::ROUNDED_CYLINDER; }

    /// Compute bounding box along the directions defined by the given rotation matrix.
    /// TODO
    ////virtual AABB GetBoundingBox(const ChMatrix33<>& rot) const override;

    virtual ChVector<> Baricenter() const override { return ChVector<>(0); }

    /// Get the cylinder radius.
    double GetRadius() const { return r; }

    /// Get the cylinder height.
    double GetHeight() const { return h; }

    /// Get the sweeping sphere radius.
    double GetSphereRadius() const { return sr; }

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    double r;   ///< cylinder radius
    double h;   ///< cylinder height
    double sr;  ///< radius of sweeping sphere
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChRoundedCylinder, 0)

}  // end namespace chrono

#endif
