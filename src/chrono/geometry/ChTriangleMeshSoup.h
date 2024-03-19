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

#ifndef CHC_TRIANGLEMESHSOUP_H
#define CHC_TRIANGLEMESHSOUP_H

#include <cmath>

#include "chrono/geometry/ChTriangleMesh.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// A basic triangle mesh: just a list of triangles (no edge connectivity info).
class ChApi ChTriangleMeshSoup : public ChTriangleMesh {
  private:
    std::vector<ChTriangle> m_triangles;  ///< triangle list

  public:
    ChTriangleMeshSoup() {}
    ChTriangleMeshSoup(const ChTriangleMeshSoup& source);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChTriangleMeshSoup* Clone() const override { return new ChTriangleMeshSoup(*this); }

    /// Create and return a ChTriangleMeshConnected from a Wavefront OBJ file.
    /// If an error occurrs during loading, an empty shared pointer is returned.
    static std::shared_ptr<ChTriangleMeshSoup> CreateFromWavefrontFile(const std::string& filename);

    /// Load from the given Wavefront .obj file
    bool LoadWavefrontMesh(std::string filename);

    /// Access the n-th triangle in mesh
    ////virtual ChTriangle& Triangle(int index) { return m_triangles[index]; }

    /// Add a triangle to this triangle mesh, by specifying the three coordinates
    virtual void AddTriangle(const ChVector3d& vertex0, const ChVector3d& vertex1, const ChVector3d& vertex2) override;

    /// Add a triangle to this triangle mesh, by specifying a ChTriangle
    virtual void AddTriangle(const ChTriangle& atriangle) override { m_triangles.push_back(atriangle); }

    /// Get the number of triangles already added to this mesh
    virtual unsigned int GetNumTriangles() const override { return (unsigned int)m_triangles.size(); }

    /// Access the n-th triangle in mesh
    virtual ChTriangle GetTriangle(unsigned int index) const override { return m_triangles[index]; }

    /// Get the list of triangles.
    std::vector<ChTriangle>& GetTriangles() { return m_triangles; }

    /// Clear all data
    virtual void Clear() override { this->m_triangles.clear(); }

    /// Transform all vertexes, by displacing and rotating (rotation  via matrix, so also scaling if needed)
    virtual void Transform(const ChVector3d displ, const ChMatrix33<> rotscale) override;

    /// Get the class type as an enum.
    virtual Type GetType() const override { return Type::TRIANGLEMESH_SOUP; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChTriangleMeshSoup, 0)

}  // end namespace chrono

#endif
