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

#ifndef CHC_CYLINDER_H
#define CHC_CYLINDER_H

#include "chrono/geometry/ChVolume.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// A cylindrical geometric object for collisions and visualization.
/// The cylinder is assumed to be aligned with the z axis of a frame with origin at the cylinder axis center.
class ChApi ChCylinder : public ChVolume {
  public:
    ChCylinder() : r(0), h(0) {}
    ChCylinder(double radius, double height) : r(radius), h(height) {}
    ChCylinder(const ChCylinder& source);
    ~ChCylinder() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChCylinder* Clone() const override { return new ChCylinder(*this); }

    /// Get the class type as an enum.
    virtual Type GetType() const override { return Type::CYLINDER; }

    /// Return the volume of this solid.
    virtual double GetVolume() const override;

    /// Return the gyration matrix for this solid.
    virtual ChMatrix33<> GetGyration() const override;

    /// Compute bounding box along the directions of the shape definition frame.
    virtual ChAABB GetBoundingBox() const override;

    /// Return the radius of a bounding sphere for this geometry.
    virtual double GetBoundingSphereRadius() const override;

    /// Compute the baricenter of the capsule.
    virtual ChVector3d Baricenter() const override { return ChVector3d(0); }

    /// Evaluate position in box volume.
    virtual ChVector3d Evaluate(double parU, double parV, double parW) const override {
        //// TODO
        return VNULL;
    }

    /// Get the cylinder radius.
    double GetRadius() const { return r; }

    /// Get the cylinder height.
    double GetHeight() const { return h; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    /// Return the volume of this type of solid with given dimensions.
    static double GetVolume(double radius, double height);

    /// Return the gyration matrix of this type of solid with given dimensions.
    static ChMatrix33<> GetGyration(double radius, double height);

    /// Return the bounding box of this type of solid with given dimensions.
    static ChAABB GetBoundingBox(double radius, double height);

    /// Return the radius of a bounding sphere.
    static double GetBoundingSphereRadius(double radius, double height);

    double r;  ///< cylinder radius
    double h;  ///< cylinder height
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChCylinder, 0)

}  // end namespace chrono

#endif
