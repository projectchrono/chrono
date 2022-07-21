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

#ifndef CHC_TRI_H
#define CHC_TRI_H

#include <cmath>

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

/// A triangle geometric shape for collisions and visualization.
class ChApi ChTriangle : public ChGeometry {
  public:
    ChVector<> p1;  ///< first triangle vertex
    ChVector<> p2;  ///< second triangle vertex
    ChVector<> p3;  ///< third triangle vertex

  public:
    ChTriangle() : p1(VNULL), p2(VNULL), p3(VNULL) {}
    ChTriangle(const ChVector<>& mp1, const ChVector<>& mp2, const ChVector<>& mp3) : p1(mp1), p2(mp2), p3(mp3) {}
    ChTriangle(const ChTriangle& source);
    ~ChTriangle() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChTriangle* Clone() const override { return new ChTriangle(*this); }

    /// Assignment operator: copy from another triangle
    ChTriangle& operator=(const ChTriangle& source);

    virtual GeometryType GetClassType() const override { return TRIANGLE; }

    /// Compute bounding box along the directions defined by the given rotation matrix.
    virtual void GetBoundingBox(ChVector<>& cmin, ChVector<>& cmax, const ChMatrix33<>& rot) const override;

    virtual ChVector<> Baricenter() const override;

    /// This is a surface
    virtual int GetManifoldDimension() const override { return 2; }

    // return false if triangle has almost zero area
    bool IsDegenerated() const;

    // compute triangle normal
    bool Normal(ChVector<>& N) const;
    ChVector<> GetNormal() const;

    /// Given point B, computes the distance from this triangle plane,
    /// returning also the projection of point on the plane.
    double PointTriangleDistance(ChVector<> B,           ///< point to be measured
                                 double& mu,             ///< returns U parametric coord of projection
                                 double& mv,             ///< returns V parametric coord of projection
                                 bool& is_into,          ///< returns true if projection falls on the triangle
                                 ChVector<>& Bprojected  ///< returns the position of the projected point
    );

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChTriangle, 0)

}  // end namespace chrono

#endif
