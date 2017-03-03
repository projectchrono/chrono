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

#ifndef CHC_TRIANGLEMESHCONNECTED_H
#define CHC_TRIANGLEMESHCONNECTED_H

#include <cmath>
#include <array>
#include <map>

#include "chrono/geometry/ChTriangleMesh.h"

namespace chrono {
namespace geometry {

/// A triangle mesh with connectivity info: vertices can be
/// shared between faces.

class ChApi ChTriangleMeshConnected : public ChTriangleMesh {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChTriangleMeshConnected)

  public:
    std::vector<ChVector<double>> m_vertices;
    std::vector<ChVector<double>> m_normals;
    std::vector<ChVector<double>> m_UV;
    std::vector<ChVector<float>> m_colors;

    std::vector<ChVector<int>> m_face_v_indices;
    std::vector<ChVector<int>> m_face_n_indices;
    std::vector<ChVector<int>> m_face_uv_indices;
    std::vector<ChVector<int>> m_face_col_indices;

    std::string m_filename;  ///< file string if loading an obj file

  public:
    ChTriangleMeshConnected() {}
    ChTriangleMeshConnected(const ChTriangleMeshConnected& source);
    ~ChTriangleMeshConnected() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChTriangleMeshConnected* Clone() const override { return new ChTriangleMeshConnected(*this); }

    std::vector<ChVector<double>>& getCoordsVertices() { return m_vertices; }
    std::vector<ChVector<double>>& getCoordsNormals() { return m_normals; }
    std::vector<ChVector<double>>& getCoordsUV() { return m_UV; }
    std::vector<ChVector<float>>& getCoordsColors() { return m_colors; }

    std::vector<ChVector<int>>& getIndicesVertexes() { return m_face_v_indices; }
    std::vector<ChVector<int>>& getIndicesNormals() { return m_face_n_indices; }
    std::vector<ChVector<int>>& getIndicesUV() { return m_face_uv_indices; }
    std::vector<ChVector<int>>& getIndicesColors() { return m_face_col_indices; }

    // Load a triangle mesh saved as a Wavefront .obj file
    void LoadWavefrontMesh(std::string filename, bool load_normals = true, bool load_uv = false);

    /// Add a triangle to this triangle mesh, by specifying the three coordinates.
    /// This is disconnected - no vertex sharing is used even if it could be..
    virtual void addTriangle(const ChVector<>& vertex0, const ChVector<>& vertex1, const ChVector<>& vertex2) override {
        int base_v = (int)m_vertices.size();
        m_vertices.push_back(vertex0);
        m_vertices.push_back(vertex1);
        m_vertices.push_back(vertex2);
        m_face_v_indices.push_back(ChVector<int>(base_v, base_v + 1, base_v + 2));
    }

    /// Add a triangle to this triangle mesh, by specifying a ChTriangle
    virtual void addTriangle(const ChTriangle& atriangle) override {
        int base_v = (int)m_vertices.size();
        m_vertices.push_back(atriangle.p1);
        m_vertices.push_back(atriangle.p2);
        m_vertices.push_back(atriangle.p3);
        m_face_v_indices.push_back(ChVector<int>(base_v, base_v + 1, base_v + 2));
    }

    /// Get the number of triangles already added to this mesh
    virtual int getNumTriangles() const override { return (int)m_face_v_indices.size(); }

    /// Access the n-th triangle in mesh
    virtual ChTriangle getTriangle(int index) const override {
        return ChTriangle(m_vertices[m_face_v_indices[index].x()], m_vertices[m_face_v_indices[index].y()],
                          m_vertices[m_face_v_indices[index].z()]);
    }

    /// Clear all data
    virtual void Clear() override {
        this->getCoordsVertices().clear();
        this->getCoordsNormals().clear();
        this->getCoordsUV().clear();
        this->getCoordsColors().clear();
        this->getIndicesVertexes().clear();
        this->getIndicesNormals().clear();
        this->getIndicesUV().clear();
        this->getIndicesColors().clear();
    }

    /// Compute barycenter, mass, inertia tensor
    void ComputeMassProperties(bool bodyCoords, double& mass, ChVector<>& center, ChMatrix33<>& inertia);

    /// Get the filename of the triangle mesh
    std::string GetFileName() { return m_filename; }

    /// Transform all vertexes, by displacing and rotating (rotation  via matrix, so also scaling if needed)
    virtual void Transform(const ChVector<> displ, const ChMatrix33<> rotscale) override;

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

    bool ComputeWingedEdges(std::map<std::pair<int, int>, std::pair<int, int>>& winged_edges,
                            bool allow_single_wing = true) const;

    /// Connect overlapping vertexes.
    /// This can beused to attempt to repair a mesh with 'open edges' to transform it into a watertight mesh.
    /// Say, if a cube is modeled with 6 faces with 4 distinct vertexes each, it might display properly, but for
    /// some algorithms, ex. collision detection, topological information might be needed, hence adjacent faces must
    /// be connected.
    /// Return the number of merged vertexes.

    int RepairDuplicateVertexes(
        const double tolerance = 1e-18  ///< when vertexes are closer than this value, they are merged
        );

    /// Offset the mesh, by a specified value, orthogonally to the faces.
    /// The offset can be inward or outward.
    /// Note: self-collisions and inverted faces resulting from excessive offsets are NOT trimmed;
    ///       so this is mostly meant to be a fast tool for making small offsets.

    bool MakeOffset(const double offset);

    /// Return the indexes of the two vertexes of the i-th edge of the triangle
    std::pair<int, int> GetTriangleEdgeIndexes (
        const std::vector<ChVector<int>>& indexes, ///< indexes, xyz per each face, ex. getIndicesVertexes()
        int it,      ///< triangle index
        int nedge,   ///< number of edge: 0,1,2 
        bool unique  ///< if true, swaps the pair so that 1st is always < 2nd id, so can test sharing wiht other triangle
        );

    /// Split a given edge by inserting a vertex in the middle: from two triangles one
    /// gets four triangles. It also interpolate normals, colors, uv. It also used and modifies the
    /// triangle neighbouring map.
    /// If the two triangles do not share an edge, returns false.
    bool SplitEdge (
        int itA,      ///< triangle A index,
        int itB,      ///< triangle B index, -1 if not existing (means free edge on A)
        int neA,      ///< n.edge on tri A: 0,1,2
        int neB,      ///< n.edge on tri B: 0,1,2
        int& itA_1,   ///< returns the index of split triangle A, part1
        int& itA_2,   ///< returns the index of split triangle A, part2
        int& itB_1,   ///< returns the index of split triangle B, part1
        int& itB_2,   ///< returns the index of split triangle B, part2
        std::vector<std::array<int, 4>>& tri_map, ///< triangle neighbouring map
        std::vector<std::vector<double>*>& aux_data_double, ///< auxiliary buffers to interpolate (assuming indexed as vertexes: each with same size as vertex buffer)
        std::vector<std::vector<int>*>& aux_data_int,       ///< auxiliary buffers to interpolate (assuming indexed as vertexes: each with same size as vertex buffer)
        std::vector<std::vector<bool>*>& aux_data_bool,      ///< auxiliary buffers to interpolate (assuming indexed as vertexes: each with same size as vertex buffer)
        std::vector<std::vector<ChVector<>>*>& aux_data_vect///< auxiliary buffers to interpolate (assuming indexed as vertexes: each with same size as vertex buffer)
        );

    /// Class to be used optionally in RefineMeshEdges()
    class ChRefineEdgeCriterion {
      public:
        virtual ~ChRefineEdgeCriterion() {}

        // Compute lenght of an edge or more in general a
        // merit function - the higher, the more likely the edge must be cut
        virtual double ComputeLength(const int vert_a, const int vert_b, ChTriangleMeshConnected* mmesh) = 0;
    };

    /// Performs mesh refinement using Rivara LEPP long-edge bisection algorithm.
    /// Given a conforming, non-degenerate triangulation, it construct a locally refined
    /// triangulation with a prescribed resolution. This algorithm, for increasing resolution,
    /// tends to produce triangles with bounded angles even if starting from skewed/skinny 
    /// triangles in the coarse mesh.
    /// Based on "Multithread parallelization of Lepp-bisection algorithms"
    ///    M.-C. Rivara et al., Applied Numerical Mathematics 62 (2012) 473–488

    void RefineMeshEdges(
        std::vector<int>& marked_tris,  ///< indexes of triangles to refine (also surrounding triangles might be affected by refinements)
        double edge_maxlen,              ///< maximum length of edge (small values give higher resolution)
        ChRefineEdgeCriterion* criterion, ///< criterion for computing lenght (or other merit function) of edge, if =0 uses default (euclidean length)
        std::vector<std::array<int, 4>>* atri_map, ///< triangle connectivity map: use and modify it. Optional. If =0, creates a temporary one just for life span of function.
        std::vector<std::vector<double>*>& aux_data_double, ///< auxiliary buffers to refine (assuming indexed as vertexes: each with same size as vertex buffer)
        std::vector<std::vector<int>*>& aux_data_int,       ///< auxiliary buffers to refine (assuming indexed as vertexes: each with same size as vertex buffer)
        std::vector<std::vector<bool>*>& aux_data_bool,      ///< auxiliary buffers to refine (assuming indexed as vertexes: each with same size as vertex buffer)
        std::vector<std::vector<ChVector<>>*>& aux_data_vect///< auxiliary buffers to refine (assuming indexed as vertexes: each with same size as vertex buffer)
        );


    virtual GeometryType GetClassType() const override { return TRIANGLEMESH_CONNECTED; }

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChTriangleMeshConnected>();
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
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChTriangleMeshConnected>();
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

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChTriangleMeshConnected,0)

}  // end namespace chrono

#endif
