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

#ifndef CHC_CONE_H
#define CHC_CONE_H

#include "chrono/geometry/ChVolume.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// A conical geometric object for collisions and visualization.
/// The cone is assumed to be aligned with the z axis of a frame with origin at the cone axis center.
class ChApi ChCone : public ChVolume {
  public:
    ChCone() : r(0), h(0) {}
    ChCone(double radius, double height) : r(radius), h(height) {}
    ChCone(const ChCone& source);
    ~ChCone() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChCone* Clone() const override { return new ChCone(*this); }

    /// Get the class type as an enum.
    virtual Type GetType() const override { return Type::CONE; }

    /// Return the volume of this solid.
    virtual double GetVolume() const override;

    /// Return the gyration matrix for this solid.
    virtual ChMatrix33<> GetGyration() const override;

    /// Compute bounding box along the directions of the shape definition frame.
    virtual ChAABB GetBoundingBox() const override;

    /// Return the radius of a bounding sphere for this geometry.
    virtual double GetBoundingSphereRadius() const override;

    /// Compute the baricenter of the cone.
    virtual ChVector3d Baricenter() const override { return ChVector3d(); }

    /// Evaluate position in box volume.
    virtual ChVector3d Evaluate(double parU, double parV, double parW) const override {
        //// TODO
        return VNULL;
    }

    /// Get the cone radius.
    double GetRadius() const { return r; }

    /// Get the cone height.
    double GetHeight() const { return h; }

    /// Return the volume of this type of solid with given dimensions.
    static double CalcVolume(double radius, double height);

    /// Return the gyration matrix of this type of solid with given dimensions.
    static ChMatrix33<> CalcGyration(double radius, double height);

    /// Return the bounding box of this type of solid with given dimensions.
    static ChAABB CalcBoundingBox(double radius, double height);

    /// Return the radius of a bounding sphere.
    static double CalcBoundingSphereRadius(double radius, double height);

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    double r;
    double h;
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChCone, 0)

}  // end namespace chrono

#endif
