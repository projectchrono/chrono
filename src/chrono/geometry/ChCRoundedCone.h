//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_ROUNDEDCONE_H
#define CHC_ROUNDEDCONE_H


#include "ChCGeometry.h"

namespace chrono {
namespace geometry {

#define EPS_SHPEREDEGENERATE 1e-20

#define CH_GEOCLASS_ROUNDEDCONE 17

///
/// A sphere.
/// Geometric object for collisions and such.
///

class ChApi ChRoundedCone : public ChGeometry {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChRoundedCone, ChGeometry);

  public:
    //
    // CONSTRUCTORS
    //

    ChRoundedCone() {
        center = VNULL;
        rad = 0;
    };

    ChRoundedCone(Vector& mc, Vector mrad) {
        center = mc;
        rad = mrad;
    }

    ChRoundedCone(const ChRoundedCone& source) { Copy(&source); }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
    void Copy(const ChRoundedCone* source) {
        center = source->center;
        rad = source->rad;
    }
#pragma GCC diagnostic pop

    ChGeometry* Duplicate() override {
        ChGeometry* mgeo = new ChRoundedCone();
        mgeo->Copy(this);
        return mgeo;
    };

    //
    // OVERRIDE BASE CLASS FUNCTIONS
    //

    virtual int GetClassType() override { return CH_GEOCLASS_ROUNDEDCONE; };

    virtual void GetBoundingBox(double& xmin,
                                double& xmax,
                                double& ymin,
                                double& ymax,
                                double& zmin,
                                double& zmax,
                                ChMatrix33<>* Rot = NULL) override {}

    virtual Vector Baricenter() override { return center; }

    virtual void CovarianceMatrix(ChMatrix33<>& C) override {}

    /// This is a solid
    virtual int GetManifoldDimension() override { return 3; }

    //
    // DATA
    //

    Vector center;

    Vector rad;
    /// Radius of sweeping sphere
    double radsphere;

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive) override
    {
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
    virtual void ArchiveIN(ChArchiveIn& marchive) override
    {
        // version number
        // int version =
        marchive.VersionRead();
        // deserialize parent class
        ChGeometry::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(center);
        marchive >> CHNVP(rad);
        marchive >> CHNVP(radsphere);
    }


};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
