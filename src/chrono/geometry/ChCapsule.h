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
    ChCapsule() : rad(0), hlen(0) {}
    ChCapsule(const ChVector<>& mcenter, double mrad, double mhlen) : rad(mrad), hlen(mhlen) {}
    ChCapsule(const ChCapsule& source);
    ~ChCapsule() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChCapsule* Clone() const override { return new ChCapsule(*this); }

    virtual GeometryType GetClassType() const override { return CAPSULE; }

    virtual void GetBoundingBox(double& xmin,
                                double& xmax,
                                double& ymin,
                                double& ymax,
                                double& zmin,
                                double& zmax,
                                ChMatrix33<>* Rot = NULL) const override;

    virtual ChVector<> Baricenter() const override { return ChVector<>(0); }

    virtual void CovarianceMatrix(ChMatrix33<>& C) const override;

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

    double rad;   ///< capsule radius
    double hlen;  ///< capsule halflength
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChCapsule, 0)

}  // end namespace chrono

#endif
