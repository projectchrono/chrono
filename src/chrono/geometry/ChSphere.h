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

#ifndef CHC_SPHERE_H
#define CHC_SPHERE_H

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

/// A spherical geometric object for collisions and visualization.
class ChApi ChSphere : public ChGeometry {
  public:
    ChSphere() : rad(0) {}
    ChSphere(const ChVector<>& mc, double mrad) : rad(mrad) {}
    ChSphere(const ChSphere& source);
    ~ChSphere() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChSphere* Clone() const override { return new ChSphere(*this); }

    virtual GeometryType GetClassType() const override { return SPHERE; }

    /// Compute bounding box along the directions defined by the given rotation matrix.
    virtual void GetBoundingBox(ChVector<>& cmin, ChVector<>& cmax, const ChMatrix33<>& rot) const override;

    /// Returns the radius of a bounding sphere for this geometry.
    virtual double GetBoundingSphereRadius() const override;

    virtual ChVector<> Baricenter() const override { return ChVector<>(0); }

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

    double rad;  ///< sphere radius
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChSphere, 0)

}  // end namespace chrono

#endif
