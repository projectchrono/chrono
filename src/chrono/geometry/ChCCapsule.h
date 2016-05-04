//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_CAPSULE_H
#define CHC_CAPSULE_H

//////////////////////////////////////////////////
//
//   ChCCapsule.h
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//             www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCGeometry.h"

namespace chrono {
namespace geometry {

#define CH_GEOCLASS_CAPSULE 14

///
/// A capsule geometric object for collision, visualization, etc.
///

class ChApi ChCapsule : public ChGeometry {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChCapsule, ChGeometry);

  public:
    //
    // CONSTRUCTORS
    //

    ChCapsule() {
        center = ChVector<>(0, 0, 0);
        rad = 0;
        hlen = 0;
    };

    ChCapsule(ChVector<>& mcenter, double mrad, double mhlen) {
        center = mcenter;
        rad = mrad;
        hlen = mhlen;
    }

    ChCapsule(const ChCapsule& source) { Copy(&source); }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
    void Copy(const ChCapsule* source) {
        center = source->center;
        rad = source->rad;
        hlen = source->hlen;
    }
#pragma GCC diagnostic pop

    ChGeometry* Duplicate() override {
        ChGeometry* mgeo = new ChCapsule();
        mgeo->Copy(this);
        return mgeo;
    }

    //
    // OVERRIDE BASE CLASS FUNCTIONS
    //

    virtual int GetClassType() override { return CH_GEOCLASS_CAPSULE; }

    virtual void GetBoundingBox(double& xmin,
                                double& xmax,
                                double& ymin,
                                double& ymax,
                                double& zmin,
                                double& zmax,
                                ChMatrix33<>* Rot = NULL) override {
        Vector trsfCenter = Rot ? Rot->MatrT_x_Vect(center) : center;

        xmin = trsfCenter.x - rad;
        xmax = trsfCenter.x + rad;
        ymin = trsfCenter.y - (rad + hlen);
        ymax = trsfCenter.y + (rad + hlen);
        zmin = trsfCenter.z - rad;
        zmax = trsfCenter.z + rad;
    }

    virtual ChVector<> Baricenter() override { return center; }

    //***TO DO***  obsolete/unused
    virtual void CovarianceMatrix(ChMatrix33<>& C) override {
        C.Reset();
        C(0, 0) = center.x * center.x;
        C(1, 1) = center.y * center.y;
        C(2, 2) = center.z * center.z;
    }

    /// This is a solid
    virtual int GetManifoldDimension() override { return 3; }

    //
    // DATA
    //

    ChVector<> center;
    double rad;
    double hlen;

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive) const override
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChGeometry::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP_OUT(center);
        marchive << CHNVP_OUT(rad);
        marchive << CHNVP_OUT(hlen);
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
        marchive >> CHNVP_IN(center);
        marchive >> CHNVP_IN(rad);
        marchive >> CHNVP_IN(hlen); 
    }

};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
