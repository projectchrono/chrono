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

/// @addtogroup chrono_geometry
/// @{

/// A box geometric object for collisions and visualization.
class ChApi ChBox : public ChVolume {
  public:
    ChBox() {}
    ChBox(const ChVector3d& lengths);
    ChBox(double length_x, double length_y, double length_z);
    ChBox(const ChBox& source);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChBox* Clone() const override { return new ChBox(*this); }

    /// Get the class type as an enum.
    virtual Type GetType() const override { return Type::BOX; }

    /// Return the volume of this solid.
    virtual double GetVolume() const override;

    /// Return the gyration matrix for this solid.
    virtual ChMatrix33<> GetGyration() const override;

    /// Compute bounding box along the directions of the shape definition frame.
    virtual ChAABB GetBoundingBox() const override;

    /// Return the radius of a bounding sphere for this geometry.
    virtual double GetBoundingSphereRadius() const override;

    /// Compute the baricenter of the box.
    virtual ChVector3d Baricenter() const override { return ChVector3d(0); }

    /// Evaluate position in box volume.
    virtual ChVector3d Evaluate(double parU, double parV, double parW) const override;

    /// Get the box half-lengths.
    const ChVector3d& GetHalflengths() const { return hlen; }

    /// Get the x, y, and z lengths of this box.
    ChVector3d GetLengths() const { return 2.0 * hlen; }

    /// Set the x, y, and z lengths of this box.
    void SetLengths(const ChVector3d& lengths) { hlen = 0.5 * lengths; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    /// Return the volume of this type of solid with given dimensions.
    static double CalcVolume(const ChVector3d& lengths);

    /// Return the gyration matrix of this type of solid with given dimensions.
    static ChMatrix33<> CalcGyration(const ChVector3d& lengths);

    /// Return the bounding box of this type of solid with given dimensions.
    static ChAABB CalcBoundingBox(const ChVector3d& lengths);

    /// Return the radius of a bounding sphere.
    static double CalcBoundingSphereRadius(const ChVector3d& lengths);

    ChVector3d hlen;  ///< box halflengths
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChBox, 0)

}  // end namespace chrono

#endif
