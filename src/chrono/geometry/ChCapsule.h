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

#ifndef CHC_CAPSULE_H
#define CHC_CAPSULE_H

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

/// A capsule geometric object for collision and visualization.
class ChApi ChCapsule : public ChGeometry {
  public:
    ChCapsule() : r(0), h(0) {}
    ChCapsule(double radius, double height) : r(radius), h(height) {}
    ChCapsule(const ChCapsule& source);
    ~ChCapsule() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChCapsule* Clone() const override { return new ChCapsule(*this); }

    /// Get the class type as an enum.
    virtual Type GetClassType() const override { return Type::CAPSULE; }

    /// Compute bounding box along the directions defined by the given rotation matrix.
    /// Note: 'rot' currently ignored.
    virtual AABB GetBoundingBox(const ChMatrix33<>& rot) const override;

    /// Returns the radius of a bounding sphere for this geometry.
    virtual double GetBoundingSphereRadius() const override;

    /// Compute the baricenter of the capsule.
    virtual ChVector<> Baricenter() const override { return ChVector<>(0); }

    /// Get the capsule radius.
    double GetRadius() const { return r; }

    /// Get the capsule height (length of cylindrical portion).
    double GetHeight() const { return h; }

    /// Get the capsule total length.
    double GetLength() const { return h + 2 * r; }

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    double r;  ///< capsule radius
    double h;  ///< height of cylindrical portion
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChCapsule, 0)

}  // end namespace chrono

#endif
