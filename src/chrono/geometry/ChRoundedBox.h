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

    /// Return the volume of this solid.
    virtual double GetVolume() const override;

    /// Return the gyration matrix for this solid.
    virtual ChMatrix33<> GetGyration() const override;

    /// Compute bounding box along the directions of the shape definition frame.
    virtual ChAABB GetBoundingBox() const override;

    /// Return the radius of a bounding sphere for this geometry.
    virtual double GetBoundingSphereRadius() const override;

    /// Computes the baricenter of the box.
    virtual ChVector<> Baricenter() const override { return ChVector<>(0); }

    /// Evaluate position in rounded box volume.
    virtual ChVector<> Evaluate(double parU, double parV, double parW) const override;

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

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    /// Return the volume of this type of solid with given dimensions.
    static double GetVolume(const ChVector<>& lengths, double srad);

    /// Return the gyration matrix of this type of solid with given dimensions.
    static ChMatrix33<> GetGyration(const ChVector<>& lengths, double srad);

    /// Return the bounding box of this type of solid with given dimensions.
    static ChAABB GetBoundingBox(const ChVector<>& lengths, double srad);

    /// Return the radius of a bounding sphere.
    static double GetBoundingSphereRadius(const ChVector<>& lengths, double srad);

    ChVector<> hlen;  ///< box halflengths
    double srad;      ///< radius of sweeping sphere
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChRoundedBox, 0)

}  // end namespace chrono

#endif
