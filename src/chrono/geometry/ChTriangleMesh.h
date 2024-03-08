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

#ifndef CHC_TRIANGLEMESH_H
#define CHC_TRIANGLEMESH_H

#include "chrono/geometry/ChTriangle.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// Base class for triangle meshes.
class ChApi ChTriangleMesh : public ChGeometry {
  public:
    ChTriangleMesh() {}
    virtual ~ChTriangleMesh() {}

    /// Add a triangle to this triangle mesh, by specifying the three coordinates
    virtual void AddTriangle(const ChVector3d& vertex0, const ChVector3d& vertex1, const ChVector3d& vertex2) = 0;

    /// Add a triangle to this triangle mesh, by specifying a ChTriangle
    virtual void AddTriangle(const ChTriangle& atriangle) = 0;

    /// Get the number of triangles already added to this mesh
    virtual unsigned int GetNumTriangles() const = 0;

    /// Get the n-th triangle in mesh
    virtual ChTriangle GetTriangle(unsigned int index) const = 0;

    /// Clear all data
    virtual void Clear() = 0;

    /// Transform all vertexes, by displacing and rotating (rotation  via matrix, so also scaling if needed)
    virtual void Transform(const ChVector3d displ, const ChMatrix33<> rotscale) = 0;

    /// Transform all vertexes, by displacing and rotating (rotation  via matrix, so also scaling if needed)
    virtual void Transform(const ChVector3d displ, const ChQuaternion<> mquat = ChQuaternion<>(1, 0, 0, 0));

    /// Get the class type as an enum.
    virtual Type GetType() const override { return Type::TRIANGLEMESH; }

    /// Compute bounding box of this triangle mesh.
    virtual ChAABB GetBoundingBox() const override;

    //// TODO
    //// virtual ChVector3d Baricenter() const override;

    /// This is a surface
    virtual int GetManifoldDimension() const override { return 2; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChTriangleMesh, 0)

}  // end namespace chrono

#endif
