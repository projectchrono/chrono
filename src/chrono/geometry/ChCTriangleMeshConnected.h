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

#ifndef CHC_TRIANGLEMESHCONNECTED_H
#define CHC_TRIANGLEMESHCONNECTED_H

//////////////////////////////////////////////////
//
//   ChCTriangleMeshConnected.h
//
//   Triangle mesh in 3d, with shared vertices
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <math.h>

#include "ChCTriangleMesh.h"
#include <array>
#include <map>

namespace chrono {
namespace geometry {

#define CH_GEOCLASS_TRIANGLEMESHCONNECTED 11

/// A triangle mesh with connectivity info: vertices can be
/// shared between faces.

class ChApi ChTriangleMeshConnected : public ChTriangleMesh {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTriangleMeshConnected, ChTriangleMesh);

    //
    // DATA
    //

    std::vector<ChVector<double> > m_vertices;
    std::vector<ChVector<double> > m_normals;
    std::vector<ChVector<double> > m_UV;
    std::vector<ChVector<float> > m_colors;

    std::vector<ChVector<int> > m_face_v_indices;
    std::vector<ChVector<int> > m_face_n_indices;
    std::vector<ChVector<int> > m_face_uv_indices;
    std::vector<ChVector<int> > m_face_col_indices;

    // file string if loading an obj file.
    std::string m_filename;

  public:
    ChTriangleMeshConnected() { m_filename = ""; };

    //
    // CUSTOM FUNCTIONS
    //

    std::vector<ChVector<double> >& getCoordsVertices() { return m_vertices; }
    std::vector<ChVector<double> >& getCoordsNormals() { return m_normals; }
    std::vector<ChVector<double> >& getCoordsUV() { return m_UV; }
    std::vector<ChVector<float> >& getCoordsColors() { return m_colors; }

    std::vector<ChVector<int> >& getIndicesVertexes() { return m_face_v_indices; }
    std::vector<ChVector<int> >& getIndicesNormals() { return m_face_n_indices; }
    std::vector<ChVector<int> >& getIndicesUV() { return m_face_uv_indices; }
    std::vector<ChVector<int> >& getIndicesColors() { return m_face_col_indices; }

    // Load a triangle mesh saved as a Wavefront .obj file
    void LoadWavefrontMesh(std::string filename, bool load_normals = true, bool load_uv = false);

    //
    // MESH INTERFACE FUNCTIONS
    //

    /// Add a triangle to this triangle mesh, by specifying the three coordinates.
    /// This is disconnected - no vertex sharing is used even if it could be..
    virtual void addTriangle(const ChVector<>& vertex0, const ChVector<>& vertex1, const ChVector<>& vertex2) {
        int base_v = (int)m_vertices.size();
        m_vertices.push_back(vertex0);
        m_vertices.push_back(vertex1);
        m_vertices.push_back(vertex2);
        m_face_v_indices.push_back(ChVector<int>(base_v, base_v + 1, base_v + 2));
    }
    /// Add a triangle to this triangle mesh, by specifying a ChTriangle
    virtual void addTriangle(const ChTriangle& atriangle) {
        int base_v = (int)m_vertices.size();
        m_vertices.push_back(atriangle.p1);
        m_vertices.push_back(atriangle.p2);
        m_vertices.push_back(atriangle.p3);
        m_face_v_indices.push_back(ChVector<int>(base_v, base_v + 1, base_v + 2));
    }

    /// Get the number of triangles already added to this mesh
    virtual int getNumTriangles() const { return (int)m_face_v_indices.size(); }

    /// Access the n-th triangle in mesh
    virtual ChTriangle getTriangle(int index) const {
        return ChTriangle(m_vertices[m_face_v_indices[index].x], m_vertices[m_face_v_indices[index].y],
                          m_vertices[m_face_v_indices[index].z]);
    }

    /// Clear all data
    virtual void Clear() {
        this->getCoordsVertices().clear();
        this->getCoordsNormals().clear();
        this->getCoordsUV().clear();
        this->getCoordsColors().clear();
        this->getIndicesVertexes().clear();
        this->getIndicesNormals().clear();
        this->getIndicesNormals().clear();
        this->getIndicesColors().clear();
    }

    /// Compute barycenter, mass, inertia tensor
    void ComputeMassProperties(bool bodyCoords, double& mass, ChVector<>& center, ChMatrix33<>& inertia);

    /// Get the filename of the triangle mesh
    std::string GetFileName() { return m_filename; }

    /// Transform all vertexes, by displacing and rotating (rotation  via matrix, so also scaling if needed)
    virtual void Transform(const ChVector<> displ, const ChMatrix33<> rotscale);

    /// Create a map of neighbouring triangles, vector of:
    /// [Ti TieA TieB TieC]
    /// (the free sides have triangle id = -1).
    /// Return false if some edge has more than 2 neighbouring triangles
    bool ComputeNeighbouringTriangleMap(std::vector<std::array<int, 4>>& tri_map) const;

    /// Create a winged edge structure, map of {key, value} as
    /// {{edgevertexA, edgevertexB}, {triangleA, triangleB}}
    /// If allow_single_wing = false, only edges with at least 2 triangles are returned. 
    ///  Else, also boundary edges with 1 triangle (the free side has triangle id = -1).
    /// Return false if some edge has more than 2 neighbouring triangles.

    bool ComputeWingedEdges(std::map<std::pair<int,int>, std::pair<int,int>>& winged_edges, bool allow_single_wing = true) const;

    /// Connect overlapping vertexes. 
    /// This can beused to attempt to repair a mesh with 'open edges' to transform it into a watertight mesh. 
    /// Say, if a cube is modeled with 6 faces with 4 distinct vertexes each, it might display properly, but for 
    /// some algorithms, ex. collision detection, topological information might be needed, hence adjacent faces must
    /// be connected. 
    /// Return the number of merged vertexes.

    int RepairDuplicateVertexes(const double tolerance = 1e-18 ///< when vertexes are closer than this value, they are merged
                                 );

    /// Offset the mesh, by a specified value, orthogonally to the faces.
    /// The offset can be inward or outward.
    /// Note: self-collisions and inverted faces resulting from excessive offsets are NOT trimmed;
    ///       so this is mostly meant to be a fast tool for making small offsets.
    
    bool MakeOffset(const double offset);


    //
    // OVERRIDE BASE CLASS FUNCTIONS
    //

    virtual int GetClassType() { return CH_GEOCLASS_TRIANGLEMESHCONNECTED; };

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
        marchive << CHNVP(m_vertices);
        marchive << CHNVP(m_normals);
        marchive << CHNVP(m_UV);
        marchive << CHNVP(m_colors);
        marchive << CHNVP(m_face_v_indices);
        marchive << CHNVP(m_face_n_indices);
        marchive << CHNVP(m_face_uv_indices);
        marchive << CHNVP(m_face_col_indices);
        marchive << CHNVP(m_filename);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChTriangleMesh::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(m_vertices);
        marchive >> CHNVP(m_normals);
        marchive >> CHNVP(m_UV);
        marchive >> CHNVP(m_colors);
        marchive >> CHNVP(m_face_v_indices);
        marchive >> CHNVP(m_face_n_indices);
        marchive >> CHNVP(m_face_uv_indices);
        marchive >> CHNVP(m_face_col_indices);
        marchive >> CHNVP(m_filename);
    }
};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
