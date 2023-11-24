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

#ifndef CHC_ROUNDEDCYLINDER_H
#define CHC_ROUNDEDCYLINDER_H

#include "chrono/geometry/ChVolume.h"

namespace chrono {
namespace geometry {

/// A rounded cylinder (sphere-swept cylinder) geometric object for collision and visualization.
class ChApi ChRoundedCylinder : public ChVolume {
  public:
    ChRoundedCylinder() : r(0), h(0), sr(0) {}
    ChRoundedCylinder(double radius, double height, double sphere_radius);
    ChRoundedCylinder(const ChRoundedCylinder& source);
    ~ChRoundedCylinder() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChRoundedCylinder* Clone() const override { return new ChRoundedCylinder(*this); }

    virtual Type GetClassType() const override { return Type::ROUNDED_CYLINDER; }

    /// Return the volume of this solid.
    virtual double GetVolume() const override;

    /// Return the gyration matrix for this solid.
    virtual ChMatrix33<> GetGyration() const override;

    /// Compute bounding box along the directions of the shape definition frame.
    virtual ChAABB GetBoundingBox() const override;

    /// Return the radius of a bounding sphere for this geometry.
    virtual double GetBoundingSphereRadius() const override;

    virtual ChVector<> Baricenter() const override { return ChVector<>(0); }

    /// Evaluate position in box volume.
    virtual ChVector<> Evaluate(double parU, double parV, double parW) const override {
        //// TODO
        return VNULL;
    }

    /// Get the cylinder radius.
    double GetRadius() const { return r; }

    /// Get the cylinder height.
    double GetHeight() const { return h; }

    /// Get the sweeping sphere radius.
    double GetSphereRadius() const { return sr; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    /// Return the volume of this type of solid with given dimensions.
    static double GetVolume(double radius, double height, double srad);

    /// Return the gyration matrix of this type of solid with given dimensions.
    static ChMatrix33<> GetGyration(double radius, double height, double srad);

    /// Return the bounding box of this type of solid with given dimensions.
    static ChAABB GetBoundingBox(double radius, double height, double srad);

    /// Return the radius of a bounding sphere.
    static double GetBoundingSphereRadius(double radius, double height, double srad);

    double r;   ///< cylinder radius
    double h;   ///< cylinder height
    double sr;  ///< radius of sweeping sphere
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChRoundedCylinder, 0)

}  // end namespace chrono

#endif
