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
    ChRoundedBox() : hlen(VNULL), srad(0) {}
    ChRoundedBox(const ChVector<>& lengths, double sphere_radius);
    ChRoundedBox(double length_x, double length_y, double length_z, double sphere_radius);
    ChRoundedBox(const ChRoundedBox& source);
    ~ChRoundedBox() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChRoundedBox* Clone() const override { return new ChRoundedBox(*this); }

    /// Get the class type as an enum.
    virtual Type GetClassType() const override { return Type::ROUNDED_BOX; }

    /// Compute bounding box along the directions defined by the given rotation matrix.
    virtual AABB GetBoundingBox(const ChMatrix33<>& rot) const override;

    /// Computes the baricenter of the box
    virtual ChVector<> Baricenter() const override { return ChVector<>(0); }

    /// Evaluate position in cube volume
    virtual void Evaluate(ChVector<>& pos, const double parU, const double parV, const double parW) const override;

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    /// Get the box half-lengths.
    const ChVector<>& GetHalflengths() const { return hlen; }

    /// Get the x, y, and z lengths of this box.
    ChVector<> GetLengths() const { return 2.0 * hlen; }

    /// Get the sweeping sphere radius.
    double GetSphereRadius() const { return srad; }

    /// Set the x, y, and z lengths of this box.
    void SetLengths(const ChVector<>& mlen) { hlen = 0.5 * mlen; }

    /// Set the sweeping sphere radius.
    void SetSphereRadius(double radius) { srad = radius; }

    /// Get the volume (assuming no scaling in Rot matrix)
    double GetVolume() { return hlen.x() * hlen.y() * hlen.z() * 8.0; };

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    ChVector<> hlen;  ///< box halflengths
    double srad;      ///< radius of sweeping sphere
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChRoundedBox, 0)

}  // end namespace chrono

#endif
