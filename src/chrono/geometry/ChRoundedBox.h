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

#ifndef CHC_ROUNDEDBOX_H
#define CHC_ROUNDEDBOX_H

#include <cmath>

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

/// A rounded box (sphere-swept box) geometric object for collisions and visualization.

class ChApi ChRoundedBox : public ChGeometry {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChRoundedBox)

  public:
    ChMatrix33<> Rot;  /// rotation of box
    ChVector<> Pos;    /// position of center
    ChVector<> Size;   /// box halflengths
    double radsphere;  ///< radius of sweeping sphere

  public:
    ChRoundedBox() : Pos(VNULL), Size(VNULL), Rot(1), radsphere(0) {}
    ChRoundedBox(ChVector<>& mpos, ChMatrix33<>& mrot, ChVector<>& mlengths, double mradsphere)
        : Pos(mpos), Size(0.5 * mlengths), Rot(mrot), radsphere(mradsphere) {}
    ChRoundedBox(ChVector<>& mC0, ChVector<>& mC1, ChVector<>& mC2, ChVector<>& mC3);
    ChRoundedBox(const ChRoundedBox& source);
    ~ChRoundedBox() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChRoundedBox* Clone() const override { return new ChRoundedBox(*this); }

    virtual GeometryType GetClassType() const override { return ROUNDED_BOX; }

    virtual void GetBoundingBox(double& xmin,
                                double& xmax,
                                double& ymin,
                                double& ymax,
                                double& zmin,
                                double& zmax,
                                ChMatrix33<>* bbRot = NULL) const override;

    /// Computes the baricenter of the box
    virtual ChVector<> Baricenter() const override { return Pos; }

    /// Computes the covariance matrix for the box
    virtual void CovarianceMatrix(ChMatrix33<>& C) const override;

    /// Evaluate position in cube volume
    virtual void Evaluate(ChVector<>& pos,
                          const double parU,
                          const double parV = 0.,
                          const double parW = 0.) const override;

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    /// Access the rotation of the box
    ChMatrix33<>* GetRotm() { return &Rot; }

    /// Access the position of the barycenter of the box
    ChVector<>& GetPos() { return Pos; }

    /// Access the size of the box: a vector with the
    /// three hemi-lengths (lengths divided by two!)
    ChVector<>& GetSize() { return Size; }

    /// Get the x y z lengths of this box (that is, double
    /// the Size values)
    ChVector<> GetLengths() { return 2.0 * Size; }

    /// Set the x y z lengths of this box (that is, double
    /// the Size values)
    void SetLengths(ChVector<>& mlen) { Size = 0.5 * mlen; }

    // Get the 8 corner points, translated and rotated
    ChVector<> GetP1() const;
    ChVector<> GetP2() const;
    ChVector<> GetP3() const;
    ChVector<> GetP4() const;
    ChVector<> GetP5() const;
    ChVector<> GetP6() const;
    ChVector<> GetP7() const;
    ChVector<> GetP8() const;

    /// Get the n-th corner point, with ipoint = 1...8
    ChVector<> GetPn(int ipoint) const;

    /// Get the volume (assuming no scaling in Rot matrix)
    double GetVolume() { return Size.x() * Size.y() * Size.z() * 8.0; };

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChRoundedBox>();
        // serialize parent class
        ChGeometry::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(Pos);
        marchive << CHNVP(Rot);
        ChVector<> Lengths = GetLengths();
        marchive << CHNVP(Lengths);  // avoid storing 'Size', i.e. half lenths, because less intuitive
        marchive << CHNVP(radsphere);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChRoundedBox>();
        // deserialize parent class
        ChGeometry::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(Pos);
        marchive >> CHNVP(Rot);
        ChVector<> Lengths;
        marchive >> CHNVP(Lengths);
        SetLengths(Lengths);
        marchive >> CHNVP(radsphere);
    }
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChRoundedBox,0)

}  // end namespace chrono

#endif
