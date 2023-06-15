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

#ifndef CHC_ELLIPSOID_H
#define CHC_ELLIPSOID_H

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

/// An ellipsoid geometric object for collisions and such.
class ChApi ChEllipsoid : public ChGeometry {
  public:
    ChEllipsoid() : rad(0) {}
    ChEllipsoid(const ChVector<>& );
    ChEllipsoid(double axis_x, double axis_y, double axis_z);
    ChEllipsoid(const ChEllipsoid& source);
    ~ChEllipsoid() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChEllipsoid* Clone() const override { return new ChEllipsoid(*this); }

    /// Get the class type as an enum.
    virtual Type GetClassType() const override { return Type::ELLIPSOID; }

    /// Get the ellipsoid semiaxes.
    const ChVector<>& GetSemiaxes() const { return rad; }

    /// Get the x, y, and z axes of this allipsoid.
    ChVector<> GetAxes() const { return 2.0 * rad; }

    /// Compute bounding box along the directions defined by the given rotation matrix.
    /// Note: 'rot' currently ignored.
    virtual AABB GetBoundingBox(const ChMatrix33<>& rot) const override;

    /// Returns the radius of a bounding sphere for this geometry.
    virtual double GetBoundingSphereRadius() const override;

    virtual ChVector<> Baricenter() const override { return ChVector<>(0); }

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    ChVector<> rad;  ///< ellipsoid semiaxes
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChEllipsoid, 0)

}  // end namespace chrono

#endif
