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

#ifndef CHC_ROUNDEDBOX_H
#define CHC_ROUNDEDBOX_H

#include <cmath>

#include "chrono/geometry/ChVolume.h"

namespace chrono {
namespace geometry {

/// A rounded box (sphere-swept box) geometric object for collisions and visualization.
class ChApi ChRoundedBox : public ChVolume {
  public:
  public:
    ChRoundedBox() : Size(VNULL), radsphere(0) {}
    ChRoundedBox(const ChVector<>& lengths, double radsphere) : Size(0.5 * lengths), radsphere(radsphere) {}
    ////ChRoundedBox(const ChVector<>& mC0, const ChVector<>& mC1, const ChVector<>& mC2, const ChVector<>& mC3);
    ChRoundedBox(const ChRoundedBox& source);
    ~ChRoundedBox() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChRoundedBox* Clone() const override { return new ChRoundedBox(*this); }

    virtual GeometryType GetClassType() const override { return ROUNDED_BOX; }

    /// Compute bounding box along the directions defined by the given rotation matrix.
    virtual void GetBoundingBox(ChVector<>& cmin, ChVector<>& cmax, const ChMatrix33<>& rot) const override;

    /// Computes the baricenter of the box
    virtual ChVector<> Baricenter() const override { return ChVector<>(0); }

    /// Evaluate position in cube volume
    virtual void Evaluate(ChVector<>& pos, const double parU, const double parV, const double parW) const override;

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

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

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

    ChVector<> Size;   /// box halflengths
    double radsphere;  ///< radius of sweeping sphere
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChRoundedBox, 0)

}  // end namespace chrono

#endif
