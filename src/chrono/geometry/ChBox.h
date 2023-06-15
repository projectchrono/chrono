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
    ChBox() {}
    ChBox(const ChVector<>& lengths);
    ChBox(double length_x, double length_y, double length_z);
    ChBox(const ChBox& source);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChBox* Clone() const override { return new ChBox(*this); }

    /// Get the class type as an enum.
    virtual Type GetClassType() const override { return Type::BOX; }

    /// Compute bounding box along the directions defined by the given rotation matrix.
    virtual AABB GetBoundingBox(const ChMatrix33<>& rot) const override;

    /// Compute the baricenter of the box.
    virtual ChVector<> Baricenter() const override { return ChVector<>(0); }

    /// Evaluate position in cube volume
    virtual void Evaluate(ChVector<>& pos, const double parU, const double parV, const double parW) const override;

    /// This is a solid
    virtual int GetManifoldDimension() const override { return 3; }

    /// Get the box half-lengths.
    const ChVector<>& GetHalflengths() const { return hlen; }

    /// Get the x, y, and z lengths of this box.
    ChVector<> GetLengths() const { return 2.0 * hlen; }

    /// Set the x, y, and z lengths of this box.
    void SetLengths(const ChVector<>& lengths) { hlen = 0.5 * lengths; }

    /// Get the volume (assuming no scaling in Rot matrix)
    double GetVolume() const { return hlen.x() * hlen.y() * hlen.z() * 8.0; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    ChVector<> hlen;  ///< box halflengths
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChBox, 0)

}  // end namespace chrono

#endif
