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
  public:
    ChTriangleMeshSoup() {}
    ChTriangleMeshSoup(const ChTriangleMeshSoup& source);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChTriangleMeshSoup* Clone() const override { return new ChTriangleMeshSoup(*this); }

    /// Get the class type as an enum.
    virtual Type GetType() const override { return Type::TRIANGLEMESH_SOUP; }

    /// Get the number of triangles already added to this mesh.
    virtual unsigned int GetNumTriangles() const override { return static_cast<unsigned int>(m_triangles.size()); }

    /// Access the n-th triangle in mesh
    virtual ChTriangle GetTriangle(unsigned int index) const override { return m_triangles[index]; }

    /// Access the list of mesh triangles.
    std::vector<ChTriangle>& GetTriangles() { return m_triangles; }

    /// Add a triangle to this triangle mesh, by specifying the three coordinates.
    virtual void AddTriangle(const ChVector3d& vertex0, const ChVector3d& vertex1, const ChVector3d& vertex2) override;

    /// Add a triangle to this triangle mesh, by specifying a ChTriangle.
    virtual void AddTriangle(const ChTriangle& triangle) override { m_triangles.push_back(triangle); }

    /// Access the n-th triangle in mesh
    ////virtual ChTriangle& Triangle(int index) { return m_triangles[index]; }

    /// Create and return a ChTriangleMeshSoup from a Wavefront OBJ file.
    /// If an error occurrs during loading, an empty shared pointer is returned.
    static std::shared_ptr<ChTriangleMeshSoup> CreateFromWavefrontFile(const std::string& filename);

    /// Load from the given Wavefront .obj file.
    /// NB: this mesh object must already be initialized.
    bool LoadWavefrontMesh(const std::string& filename);

    /// Create and return a ChTriangleMeshSoup from an STL file.
    /// If an error occurrs during loading, an empty shared pointer is returned.
    static std::shared_ptr<ChTriangleMeshSoup> CreateFromSTLFile(const std::string& filename);

    /// Load an STL file into this triangle mesh.
    /// NB: this mesh object must already be initialized.
    bool LoadSTLMesh(const std::string& filename);

    /// Clear all data.
    virtual void Clear() override { m_triangles.clear(); }

    /// Transform all vertices, by displacing and rotating (rotation via matrix, so also scaling if needed).
    virtual void Transform(const ChVector3d& displ, const ChMatrix33d& rotscale) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    std::vector<ChTriangle> m_triangles;  ///< triangle list
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChTriangleMeshSoup, 0)

}  // end namespace chrono

#endif
