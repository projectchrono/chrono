//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_TRIANGLEMESHSOUP_H
#define CHC_TRIANGLEMESHSOUP_H


#include <math.h>

#include "ChCTriangleMesh.h"

namespace chrono {
namespace geometry {

#define CH_GEOCLASS_TRIANGLEMESHSOUP 12

///
/// A basic triangle mesh: just a list of triangles (no edge connectivity info).
///

class ChApi ChTriangleMeshSoup : public ChTriangleMesh {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTriangleMeshSoup, ChTriangleMesh);

    //
    // DATA
    //

    std::vector<ChTriangle> m_triangles;

  public:
    ChTriangleMeshSoup(){};

    /// Access the n-th triangle in mesh
    virtual ChTriangle& Triangle(int index) { return m_triangles[index]; }

    //
    // MESH INTERFACE FUNCTIONS
    //

    /// Add a triangle to this triangle mesh, by specifying the three coordinates
    virtual void addTriangle(const ChVector<>& vertex0, const ChVector<>& vertex1, const ChVector<>& vertex2) {
        ChTriangle tri(vertex0, vertex1, vertex2);
        m_triangles.push_back(tri);
    }
    /// Add a triangle to this triangle mesh, by specifying a ChTriangle
    virtual void addTriangle(const ChTriangle& atriangle) { m_triangles.push_back(atriangle); }

    /// Get the number of triangles already added to this mesh
    virtual int getNumTriangles() const { return (int)m_triangles.size(); }

    /// Access the n-th triangle in mesh
    virtual ChTriangle getTriangle(int index) const { return m_triangles[index]; }

    /// Clear all data
    virtual void Clear() { this->m_triangles.clear(); }

    /// Transform all vertexes, by displacing and rotating (rotation  via matrix, so also scaling if needed)
    virtual void Transform(const ChVector<> displ, const ChMatrix33<> rotscale);

    //
    // OVERRIDE BASE CLASS FUNCTIONS
    //

    virtual int GetClassType() { return CH_GEOCLASS_TRIANGLEMESHSOUP; };

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChTriangleMesh::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(m_triangles);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChTriangleMesh::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(m_triangles);
    }
};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
