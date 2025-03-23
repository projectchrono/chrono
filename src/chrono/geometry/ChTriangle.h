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

/// @addtogroup chrono_geometry
/// @{

/// A triangle geometric shape for collisions and visualization.
class ChApi ChTriangle : public ChGeometry {
  public:
    ChTriangle() : p1(VNULL), p2(VNULL), p3(VNULL) {}
    ChTriangle(const ChVector3d& P1, const ChVector3d& P2, const ChVector3d& P3) : p1(P1), p2(P2), p3(P3) {}
    ChTriangle(const ChTriangle& source);
    ~ChTriangle() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChTriangle* Clone() const override { return new ChTriangle(*this); }

    /// Assignment operator: copy from another triangle
    ChTriangle& operator=(const ChTriangle& source);

    /// Get the class type as an enum.
    virtual Type GetType() const override { return Type::TRIANGLE; }

    /// Compute bounding box of this triangle.
    virtual ChAABB GetBoundingBox() const override;

    /// Compute center of mass.
    virtual ChVector3d Baricenter() const override;

    /// Indicate that a triangle is a 2D surface.
    virtual int GetManifoldDimension() const override { return 2; }

    /// Return false if triangle has almost zero area.
    bool IsDegenerated() const;

    /// Compute triangle normal in provided vector and return 'false' if the triangle is degenerated.
    bool Normal(ChVector3d& N) const;

    /// Compute the triangle normal.
    ChVector3d GetNormal() const;

    /// Given point B, computes the distance from this triangle plane,
    /// returning also the projection of point on the plane.
    double PointTriangleDistance(ChVector3d B,           ///< point to be measured
                                 double& mu,             ///< returns U parametric coord of projection
                                 double& mv,             ///< returns V parametric coord of projection
                                 bool& is_into,          ///< returns true if projection falls on the triangle
                                 ChVector3d& Bprojected  ///< returns the position of the projected point
    );

    /// Set the triangle vertices.
    void SetPoints(const ChVector3d& P1, const ChVector3d& P2, const ChVector3d& P3);

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    /// Return the bounding box of a triangle with given vertices.
    static ChAABB GetBoundingBox(const ChVector3d& P1, const ChVector3d& P2, const ChVector3d& P3);

    /// Static utility function to calculate the normal of a triangle in the provided output vector.
    /// Return 'false' if the triangle is degenerated.
    static bool CalcNormal(const ChVector3d& p1, const ChVector3d& p2, const ChVector3d& p3, ChVector3d& N);

    /// Static utility function to calculate the normal of a triangle.
    static ChVector3d CalcNormal(const ChVector3d& p1, const ChVector3d& p2, const ChVector3d& p3);

    ChVector3d p1;  ///< first triangle vertex
    ChVector3d p2;  ///< second triangle vertex
    ChVector3d p3;  ///< third triangle vertex
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChTriangle, 0)

}  // end namespace chrono

#endif
