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

#ifndef CHC_CYLINDER_H
#define CHC_CYLINDER_H

//////////////////////////////////////////////////
//
//   ChCCylinder.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCGeometry.h"

namespace chrono {
namespace geometry {

#define EPS_CYLINDERDEGENERATE 1e-20

#define CH_GEOCLASS_CYLINDER 13

///
/// A cylinder.
/// Geometric object for collisions and such.
///

class ChApi ChCylinder : public ChGeometry {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChCylinder, ChGeometry);

  public:
    //
    // CONSTRUCTORS
    //

    ChCylinder() {
        p1 = VNULL;
        p2 = ChVector<>(0, 1, 0);
        rad = 0.1;
    };

    ChCylinder(ChVector<>& mp1, ChVector<>& mp2, double mrad) {
        p1 = mp1;
        p2 = mp2;
        rad = mrad;
    }

    ChCylinder(const ChCylinder& source) { Copy(&source); }

    void Copy(const ChCylinder* source) {
        p1 = source->p1;
        p2 = source->p2;
        rad = source->rad;
    };

    ChGeometry* Duplicate() {
        ChGeometry* mgeo = new ChCylinder();
        mgeo->Copy(this);
        return mgeo;
    };

    //
    // OVERRIDE BASE CLASS FUNCTIONS
    //

    virtual int GetClassType() { return CH_GEOCLASS_CYLINDER; };

    virtual void GetBoundingBox(double& xmin,
                                double& xmax,
                                double& ymin,
                                double& ymax,
                                double& zmin,
                                double& zmax,
                                ChMatrix33<>* Rot = NULL) {
        Vector dims = Vector(rad, p2.y - p1.y, rad);
        Vector trsfCenter = Baricenter();
        if (Rot) {
            trsfCenter = Rot->MatrT_x_Vect(Baricenter());
        }
        xmin = trsfCenter.x - dims.x;
        xmax = trsfCenter.x + dims.x;
        ymin = trsfCenter.y - dims.y;
        ymax = trsfCenter.y + dims.y;
        zmin = trsfCenter.z - dims.z;
        zmax = trsfCenter.z + dims.z;
    }

    virtual ChVector<> Baricenter() { return (p1 + p2) * 0.5; };

    //***TO DO***  obsolete/unused
    virtual void CovarianceMatrix(ChMatrix33<>& C) {
        C.Reset();
        C(0, 0) = p1.x * p1.x;
        C(1, 1) = p1.y * p1.y;
        C(2, 2) = p1.z * p1.z;
    };

    /// This is a solid
    virtual int GetManifoldDimension() { return 3; }

    //
    // DATA
    //

    ChVector<> p1;
    ChVector<> p2;

    double rad;

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChGeometry::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(p1);
        marchive << CHNVP(p2);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChGeometry::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(p1);
        marchive >> CHNVP(p2);
    }

};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
