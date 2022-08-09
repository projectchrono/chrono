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
    ChVector<> p1;  ///< center of first base
    ChVector<> p2;  ///< center of second base
    double rad;     ///< cylinder radius

  public:
    ChCylinder() : p1(VNULL), p2(ChVector<>(0, 1, 0)), rad(0.1) {}
    ChCylinder(const ChVector<>& mp1, const ChVector<>& mp2, double mrad) : p1(mp1), p2(mp2), rad(mrad) {}
    ChCylinder(const ChCylinder& source);
    ~ChCylinder() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChCylinder* Clone() const override { return new ChCylinder(*this); }

    virtual GeometryType GetClassType() const override { return CYLINDER; }

    /// Compute bounding box along the directions defined by the given rotation matrix.
    virtual void GetBoundingBox(ChVector<>& cmin, ChVector<>& cmax, const ChMatrix33<>& rot) const override;

    virtual ChVector<> Baricenter() const override { return (p1 + p2) * 0.5; }

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChCylinder, 0)

}  // end namespace chrono

#endif
