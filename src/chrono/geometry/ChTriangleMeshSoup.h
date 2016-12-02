// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#include <math.h>

#include "chrono/geometry/ChTriangleMesh.h"

namespace chrono {
namespace geometry {

/// A basic triangle mesh: just a list of triangles (no edge connectivity info).

class ChApi ChTriangleMeshSoup : public ChTriangleMesh {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChTriangleMeshSoup)

    std::vector<ChTriangle> m_triangles;  ///< triangle list

  public:
    ChTriangleMeshSoup() {}
    ChTriangleMeshSoup(const ChTriangleMeshSoup& source);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChTriangleMeshSoup* Clone() const override { return new ChTriangleMeshSoup(*this); }

    /// Access the n-th triangle in mesh
    virtual ChTriangle& Triangle(int index) { return m_triangles[index]; }

    /// Add a triangle to this triangle mesh, by specifying the three coordinates
    virtual void addTriangle(const ChVector<>& vertex0, const ChVector<>& vertex1, const ChVector<>& vertex2) override {
        ChTriangle tri(vertex0, vertex1, vertex2);
        m_triangles.push_back(tri);
    }

    /// Add a triangle to this triangle mesh, by specifying a ChTriangle
    virtual void addTriangle(const ChTriangle& atriangle) override { m_triangles.push_back(atriangle); }

    /// Get the number of triangles already added to this mesh
    virtual int getNumTriangles() const { return (int)m_triangles.size(); }

    /// Access the n-th triangle in mesh
    virtual ChTriangle getTriangle(int index) const { return m_triangles[index]; }

    /// Clear all data
    virtual void Clear() { this->m_triangles.clear(); }

    /// Transform all vertexes, by displacing and rotating (rotation  via matrix, so also scaling if needed)
    virtual void Transform(const ChVector<> displ, const ChMatrix33<> rotscale);

    virtual GeometryType GetClassType() const override { return TRIANGLEMESH_SOUP; }

    /// Method to allow de serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChTriangleMesh::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(m_triangles);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChTriangleMesh::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(m_triangles);
    }
};

}  // end namespace geometry
}  // end namespace chrono

#endif
