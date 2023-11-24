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

#ifndef CHC_ELLIPSOID_H
#define CHC_ELLIPSOID_H

#include "chrono/geometry/ChVolume.h"

namespace chrono {
namespace geometry {

/// An ellipsoid geometric object for collisions and such.
class ChApi ChEllipsoid : public ChVolume {
  public:
    ChEllipsoid() : rad(0) {}
    ChEllipsoid(const ChVector<>&);
    ChEllipsoid(double axis_x, double axis_y, double axis_z);
    ChEllipsoid(const ChEllipsoid& source);
    ~ChEllipsoid() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChEllipsoid* Clone() const override { return new ChEllipsoid(*this); }

    /// Get the class type as an enum.
    virtual Type GetClassType() const override { return Type::ELLIPSOID; }

    /// Get the ellipsoid semiaxes.
    const ChVector<>& GetSemiaxes() const { return rad; }

    /// Get the x, y, and z axes of this allipsoid.
    ChVector<> GetAxes() const { return 2.0 * rad; }

    /// Return the volume of this solid.
    virtual double GetVolume() const override;

    /// Return the gyration matrix for this solid.
    virtual ChMatrix33<> GetGyration() const override;

    /// Compute bounding box along the directions of the shape definition frame.
    virtual ChAABB GetBoundingBox() const override;

    /// Returns the radius of a bounding sphere for this geometry.
    virtual double GetBoundingSphereRadius() const override;

    virtual ChVector<> Baricenter() const override { return ChVector<>(0); }

    /// Evaluate position in box volume.
    virtual ChVector<> Evaluate(double parU, double parV, double parW) const override {
        //// TODO
        return VNULL;
    }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    /// Return the volume of this type of solid with given dimensions.
    static double GetVolume(const ChVector<>& axes);

    /// Return the gyration matrix of this type of solid with given dimensions.
    static ChMatrix33<> GetGyration(const ChVector<>& axes);

    /// Return the bounding box of this type of solid with given dimensions.
    static ChAABB GetBoundingBox(const ChVector<>& axes);

    /// Return the radius of a bounding sphere.
    static double GetBoundingSphereRadius(const ChVector<>& axes);

    ChVector<> rad;  ///< ellipsoid semiaxes
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChEllipsoid, 0)

}  // end namespace chrono

#endif
