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

#ifndef CHC_ROUNDEDCONE_H
#define CHC_ROUNDEDCONE_H

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

/// A rounded cone (sphere-swept cone) geometric object for collisions and visualization.

class ChApi ChRoundedCone : public ChGeometry {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChRoundedCone)

  public:
    ChRoundedCone() : center(VNULL), rad(0) {}
    ChRoundedCone(ChVector<>& mc, ChVector<> mrad) : center(mc), rad(mrad) {}
    ChRoundedCone(const ChRoundedCone& source);
    ~ChRoundedCone() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChRoundedCone* Clone() const override { return new ChRoundedCone(*this); }

    virtual GeometryType GetClassType() const override { return ROUNDED_CONE; }

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

    virtual void CovarianceMatrix(ChMatrix33<>& C) const override {
        //// TODO
    }

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    ChVector<> center;  ///< base center
    ChVector<> rad;     ///< cone radius
    double radsphere;   ///< radius of sweeping sphere

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChRoundedCone>();
        // serialize parent class
        ChGeometry::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(center);
        marchive << CHNVP(rad);
        marchive << CHNVP(radsphere);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChRoundedCone>();
        // deserialize parent class
        ChGeometry::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(center);
        marchive >> CHNVP(rad);
        marchive >> CHNVP(radsphere);
    }
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChRoundedCone,0)

}  // end namespace chrono

#endif
