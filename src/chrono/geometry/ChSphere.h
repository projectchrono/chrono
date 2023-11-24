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

#ifndef CHC_SPHERE_H
#define CHC_SPHERE_H

#include "chrono/geometry/ChVolume.h"

namespace chrono {
namespace geometry {

/// A spherical geometric object for collisions and visualization.
class ChApi ChSphere : public ChVolume {
  public:
    ChSphere() : rad(0) {}
    ChSphere(double radius) : rad(radius) {}
    ChSphere(const ChSphere& source);
    ~ChSphere() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChSphere* Clone() const override { return new ChSphere(*this); }

    /// Get the class type as an enum.
    virtual Type GetClassType() const override { return Type::SPHERE; }

    /// Get the sphere radius.
    double GetRadius() const { return rad; }

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

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    /// Return the volume of this type of solid with given dimensions.
    static double GetVolume(double radius);

    /// Return the gyration matrix of this type of solid with given dimensions.
    static ChMatrix33<> GetGyration(double radius);

    /// Return the bounding box of this type of solid with given dimensions.
    static ChAABB GetBoundingBox(double radius);

    /// Return the radius of a bounding sphere.
    static double GetBoundingSphereRadius(double radius);

    double rad;  ///< sphere radius
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChSphere, 0)

}  // end namespace chrono

#endif
