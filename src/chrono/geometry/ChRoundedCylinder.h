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

#ifndef CHC_ROUNDEDCYLINDER_H
#define CHC_ROUNDEDCYLINDER_H

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

/// A rounded cylinder (sphere-swept cylinder) geometric object for collision and visualization.

class ChApi ChRoundedCylinder : public ChGeometry {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChRoundedCylinder)

  public:
    ChVector<> center;  ///< cylinder center
    double rad;         ///< cylinder radius
    double hlen;        ///< cylinder halflength
    double radsphere;   ///< Radius of sweeping sphere

  public:
    ChRoundedCylinder() : center(VNULL), rad(0), hlen(0), radsphere(0) {}
    ChRoundedCylinder(ChVector<>& mcenter, double mrad, double mhlen, double mradsphere)
        : center(mcenter), rad(mrad), hlen(mhlen), radsphere(mradsphere) {}
    ChRoundedCylinder(const ChRoundedCylinder& source);
    ~ChRoundedCylinder() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChRoundedCylinder* Clone() const override { return new ChRoundedCylinder(*this); }

    virtual GeometryType GetClassType() const override { return ROUNDED_CYLINDER; }

    virtual void GetBoundingBox(double& xmin,
                                double& xmax,
                                double& ymin,
                                double& ymax,
                                double& zmin,
                                double& zmax,
                                ChMatrix33<>* Rot = NULL) const override {
        //// TODO
    }

    virtual ChVector<> Baricenter() const override { return center; }

    //// TODO obsolete/unused
    virtual void CovarianceMatrix(ChMatrix33<>& C) const override;

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChRoundedCylinder>();
        // serialize parent class
        ChGeometry::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(center);
        marchive << CHNVP(rad);
        marchive << CHNVP(hlen);
        marchive << CHNVP(radsphere);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChRoundedCylinder>();
        // deserialize parent class
        ChGeometry::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(center);
        marchive >> CHNVP(rad);
        marchive >> CHNVP(hlen);
        marchive >> CHNVP(radsphere);
    }
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChRoundedCylinder,0)

}  // end namespace chrono

#endif
