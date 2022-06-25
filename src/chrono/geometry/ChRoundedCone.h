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

#ifndef CHC_ROUNDEDCONE_H
#define CHC_ROUNDEDCONE_H

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

/// A rounded cone (sphere-swept cone) geometric object for collisions and visualization.
class ChApi ChRoundedCone : public ChGeometry {
  public:
    ChRoundedCone() : center(VNULL), rad(0), radsphere(0) {}
    ChRoundedCone(const ChVector<>& mc, const ChVector<>& mrad, double mradsphere)
        : center(mc), rad(mrad), radsphere(mradsphere) {}
    ChRoundedCone(const ChRoundedCone& source);
    ~ChRoundedCone() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChRoundedCone* Clone() const override { return new ChRoundedCone(*this); }

    virtual GeometryType GetClassType() const override { return ROUNDED_CONE; }

    /// Compute bounding box along the directions defined by the given rotation matrix.
    /// TODO
    ////virtual void GetBoundingBox(ChVector<>& cmin, ChVector<>& cmax, const ChMatrix33<>& rot) const override;

    virtual ChVector<> Baricenter() const override { return center; }

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    ChVector<> center;  ///< base center
    ChVector<> rad;     ///< cone radius
    double radsphere;   ///< radius of sweeping sphere

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChRoundedCone, 0)

}  // end namespace chrono

#endif
