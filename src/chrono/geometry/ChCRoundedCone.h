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

#include "chrono/geometry/ChCGeometry.h"

namespace chrono {
namespace geometry {

#define CH_GEOCLASS_ROUNDEDCONE 17

/// A rounded cone (sphere-swept cone) geometric object for collisions and visualization.

class ChApi ChRoundedCone : public ChGeometry {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChRoundedCone, ChGeometry);

  public:
    ChRoundedCone() : center(VNULL), rad(0) {}
    ChRoundedCone(ChVector<>& mc, ChVector<> mrad) : center(mc), rad(mrad) {}
    ChRoundedCone(const ChRoundedCone& source);
    ~ChRoundedCone() {}

    void Copy(const ChRoundedCone* source) {
        center = source->center;
        rad = source->rad;
    }

    ChGeometry* Duplicate() {
        ChGeometry* mgeo = new ChRoundedCone();
        mgeo->Copy(this);
        return mgeo;
    }

    virtual int GetClassType() const override { return CH_GEOCLASS_ROUNDEDCONE; }

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
        marchive.VersionWrite(1);
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
        int version = marchive.VersionRead();
        // deserialize parent class
        ChGeometry::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(center);
        marchive >> CHNVP(rad);
        marchive >> CHNVP(radsphere);
    }
};

}  // end namespace geometry
}  // end namespace chrono

#endif
