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

#ifndef CHC_CYLINDER_H
#define CHC_CYLINDER_H

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

/// A cylindrical geometric object for collisions and visualization.

class ChApi ChCylinder : public ChGeometry {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChCylinder)

  public:
    ChVector<> p1;  ///< center of first base
    ChVector<> p2;  ///< center of second base
    double rad;     ///< cylinder radius

  public:
    ChCylinder() : p1(VNULL), p2(ChVector<>(0, 1, 0)), rad(0.1) {}
    ChCylinder(ChVector<>& mp1, ChVector<>& mp2, double mrad) : p1(mp1), p2(mp2), rad(mrad) {}
    ChCylinder(const ChCylinder& source);
    ~ChCylinder() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChCylinder* Clone() const override { return new ChCylinder(*this); }

    virtual GeometryType GetClassType() const override { return CYLINDER; }

    virtual void GetBoundingBox(double& xmin,
                                double& xmax,
                                double& ymin,
                                double& ymax,
                                double& zmin,
                                double& zmax,
                                ChMatrix33<>* Rot = NULL) const override;

    virtual ChVector<> Baricenter() const override { return (p1 + p2) * 0.5; }

    virtual void CovarianceMatrix(ChMatrix33<>& C) const override;

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChCylinder>();
        // serialize parent class
        ChGeometry::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(p1);
        marchive << CHNVP(p2);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChCylinder>();
        // deserialize parent class
        ChGeometry::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(p1);
        marchive >> CHNVP(p2);
    }
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChCylinder,0)

}  // end namespace chrono

#endif
