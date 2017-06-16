// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChEllipsoid)

  public:
    ChVector<> center;  ///< ellipsoid center
    ChVector<> rad;     ///< ellipsoid semi-axes

  public:
    ChEllipsoid() : center(VNULL), rad(0) {}
    ChEllipsoid(ChVector<>& mc, ChVector<> mrad) : center(mc), rad(mrad) {}
    ChEllipsoid(const ChEllipsoid& source);
    ~ChEllipsoid() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChEllipsoid* Clone() const override { return new ChEllipsoid(*this); }

    virtual GeometryType GetClassType() const override { return SPHERE; }

    virtual void GetBoundingBox(double& xmin,
                                double& xmax,
                                double& ymin,
                                double& ymax,
                                double& zmin,
                                double& zmax,
                                ChMatrix33<>* Rot = NULL) const override;

    virtual ChVector<> Baricenter() const override { return center; }

    virtual void CovarianceMatrix(ChMatrix33<>& C) const override;

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChEllipsoid>();
        // serialize parent class
        ChGeometry::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(center);
        marchive << CHNVP(rad);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChEllipsoid>();
        // deserialize parent class
        ChGeometry::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(center);
        marchive >> CHNVP(rad);
    }
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChEllipsoid,0)

}  // end namespace chrono

#endif
