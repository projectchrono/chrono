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

#ifndef CHC_BOX_H
#define CHC_BOX_H

#include <cmath>

#include "chrono/geometry/ChVolume.h"

namespace chrono {
namespace geometry {

/// A box geometric object for collisions and visualization.
class ChApi ChBox : public ChVolume {
  public:
    ChBox() : Size(VNULL) {}
    ChBox(const ChVector<>& lengths);
    ChBox(double len_x, double len_y, double len_z);
    ////ChBox(const ChVector<>& mC0, const ChVector<>& mC1, const ChVector<>& mC2, const ChVector<>& mC3);
    ChBox(const ChBox& source);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChBox* Clone() const override { return new ChBox(*this); }

    virtual ChGeometry::GeometryType GetClassType() const override { return BOX; }

    virtual void GetBoundingBox(double& xmin,
                                double& xmax,
                                double& ymin,
                                double& ymax,
                                double& zmin,
                                double& zmax,
                                ChMatrix33<>* bbRot = NULL) const override;

    /// Computes the baricenter of the box
    virtual ChVector<> Baricenter() const override { return ChVector<>(0); }

    /// Computes the covariance matrix for the box
    virtual void CovarianceMatrix(ChMatrix33<>& C) const override;

    /// Evaluate position in cube volume
    virtual void Evaluate(ChVector<>& pos, const double parU, const double parV, const double parW) const override;

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    /// Get the box half-lengths
    ChVector<>& GetSize() { return Size; }

    /// Get the x y z lengths of this box (that is, double the Size values)
    ChVector<> GetLengths() { return 2.0 * Size; }

    /// Set the x y z lengths of this box (that is, double
    /// the Size values)
    void SetLengths(const ChVector<>& mlen) { Size = 0.5 * mlen; }

    // Get the 8 corner points.
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
    double GetVolume() const { return Size.x() * Size.y() * Size.z() * 8.0; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

    ChVector<> Size;  ///< box halflengths
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChBox, 0)

}  // end namespace chrono

#endif
