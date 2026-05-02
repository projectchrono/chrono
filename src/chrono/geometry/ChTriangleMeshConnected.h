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

#ifndef CHC_TRIANGLEMESHCONNECTED_H
#define CHC_TRIANGLEMESHCONNECTED_H

#include <array>
#include <cmath>
#include <map>

#include "chrono/assets/ChColor.h"
#include "chrono/core/ChVector2.h"
#include "chrono/physics/ChMassProperties.h"
#include "chrono/geometry/ChProperty.h"
#include "chrono/geometry/ChTriangleMesh.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// A triangle mesh with connectivity info: vertices can be shared between faces.
/// To keep this simple, the class assumes that you will manage the size of vectors
/// m_vertices, m_normals etc., so be cautious about resizing, etc.
/// We assume that
/// - if no m_face_uv_indices but m_UV.size() == m_vertices.size(), then m_UV represents per-vertex UV, otherwise
/// per-face-corner
/// - if no m_face_col_indices but m_colors.size() == m_vertices.size(), then m_colors represents per-vertex colors,
/// otherwise per-face-corner.
class ChApi ChTriangleMeshConnected : public ChTriangleMesh {
  public:
    ChTriangleMeshConnected() {}
    ChTriangleMeshConnected(const ChTriangleMeshConnected& source);
    ~ChTriangleMeshConnected();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChTriangleMeshConnected* Clone() const override { return new ChTriangleMeshConnected(*this); }

    /// Data accessors ---------------------------------------------------------

    /// Get the class type as an enum.
    virtual Type GetType() const override { return Type::TRIANGLEMESH_CONNECTED; }

    /// Access triangle mesh data.
    std::vector<ChVector3d>& GetCoordsVertices() { return m_vertices; }
    std::vector<ChVector3d>& GetCoordsNormals() { return m_normals; }
    std::vector<ChVector2d>& GetCoordsUV() { return m_UV; }
    std::vector<ChColor>& GetCoordsColors() { return m_colors; }
    std::vector<ChVector3i>& GetIndicesVertices() { return m_face_v_indices; }
    std::vector<ChVector3i>& GetIndicesNormals() { return m_face_n_indices; }
    std::vector<ChVector3i>& GetIndicesUV() { return m_face_uv_indices; }
    std::vector<ChVector3i>& GetIndicesColors() { return m_face_col_indices; }
    std::vector<int>& GetIndicesMaterials() { return m_face_mat_indices; }
    std::vector<ChProperty*>& GetPropertiesPerVertex() { return m_properties_per_vertex; }
    std::vector<ChProperty*>& GetPropertiesPerFace() { return m_properties_per_face; };

    /// Get triangle mesh data.
    const std::vector<ChVector3d>& GetCoordsVertices() const { return m_vertices; }
    const std::vector<ChVector3d>& GetCoordsNormals() const { return m_normals; }
    const std::vector<ChVector2d>& GetCoordsUV() const { return m_UV; }
    const std::vector<ChColor>& GetCoordsColors() const { return m_colors; }
    const std::vector<ChVector3i>& GetIndicesVertices() const { return m_face_v_indices; }
    const std::vector<ChVector3i>& GetIndicesNormals() const { return m_face_n_indices; }
    const std::vector<ChVector3i>& GetIndicesUV() const { return m_face_uv_indices; }
    const std::vector<ChVector3i>& GetIndicesColors() const { return m_face_col_indices; }
    const std::vector<int>& GetIndicesMaterials() const { return m_face_mat_indices; }

    /// Get faces data.
    const std::vector<ChVector3d>& GetFaceVertices();
    const std::vector<ChVector3d>& GetFaceNormals();
    const std::vector<ChColor>& GetFaceColors();
    const std::vector<ChVector3d>& GetAverageNormals();

    /// Get the number of vertices in this mesh.
    unsigned int GetNumVertices() const { return static_cast<unsigned int>(m_vertices.size()); }

    /// Get the number of normals in this mesh.
    unsigned int GetNumNormals() const { return static_cast<unsigned int>(m_normals.size()); }

    /// Get the number of triangles already added to this mesh.
    virtual unsigned int GetNumTriangles() const override { return static_cast<unsigned int>(m_face_v_indices.size()); }

    /// Get the n-th triangle in mesh.
    /// Note: the triangle is created on-the-fly.
    virtual ChTriangle GetTriangle(unsigned int index) const override;

    /// Get the filename of the triangle mesh.
    const std::string& GetFileName() const { return m_filename; }

    /// Compute bounding box of this triangle mesh.
    virtual ChAABB GetBoundingBox() const override;

    /// Return the bounding box of a triangle mesh with given vertices.
    static ChAABB CalcBoundingBox(const std::vector<ChVector3d>& vertices);

    /// Return the indices of the two vertices of the i-th edge of the triangle.
    /// If unique=true, swap the pair so that 1st < 2nd, to permit test sharing with other triangle.
    std::pair<int, int> GetTriangleEdgeIndices(const ChVector3i& face_indices,  ///< indices of a triangular face
                                               int num_edge,                    ///< number of edge: 0, 1, 2
                                               bool unique                      ///< swap?
    );

    /// Data population --------------------------------------------------------

    /// Add a triangle to this triangle mesh, by specifying the three coordinates.
    /// NB: This is disconnected - no vertex sharing is used even if it could be.
    virtual void AddTriangle(const ChVector3d& vertex0, const ChVector3d& vertex1, const ChVector3d& vertex2) override;

    /// Add a triangle to this triangle mesh, by specifying a ChTriangle.
    virtual void AddTriangle(const ChTriangle& triangle) override;

    /// Create and return a ChTriangleMeshConnected from a Wavefront OBJ file.
    /// If an error occurs during loading, an empty shared pointer is returned.
    static std::shared_ptr<ChTriangleMeshConnected> CreateFromWavefrontFile(const std::string& filename, bool load_normals = true, bool load_uv = false);

    /// Load a Wavefront OBJ file into this triangle mesh.
    /// NB: this mesh object must already be initialized.
    bool LoadWavefrontMesh(const std::string& filename, bool load_normals = true, bool load_uv = false);

    /// Write the specified meshes in a Wavefront OBJ file.
    static bool WriteWavefront(const std::string& filename, const std::vector<ChTriangleMeshConnected>& meshes);

    /// Create and return a ChTriangleMeshConnected from an STL file.
    /// If an error occurs during loading, an empty shared pointer is returned.
    static std::shared_ptr<ChTriangleMeshConnected> CreateFromSTLFile(const std::string& filename, bool load_normals = true);

    /// Load an STL file into this triangle mesh.
    /// NB: this mesh object must already be initialized.
    bool LoadSTLMesh(const std::string& filename, bool load_normals = true);

    /// Write the specified mesh in a binary STL file.
    static bool WriteSTL(const std::string& filename, const ChTriangleMeshConnected& trimesh);

    /// Add a property as an array of data per-vertex. Deletion will be automatic at the end of mesh life.
    /// Warning: mprop.data.size() must be equal to m_vertices.size(). Cost: allocation and a data copy.
    void AddPropertyPerVertex(ChProperty& prop) { m_properties_per_vertex.push_back(prop.clone()); }

    /// Add a property as an array of data per-face. Deletion will be automatic at the end of mesh life.
    /// Warning: mprop.data.size() must be equal to m_vertices.size(). Cost: allocation a data copy.
    void AddPropertyPerFace(ChProperty& prop) { m_properties_per_face.push_back(prop.clone()); }

    /// Clear all data.
    virtual void Clear() override;

    /// Mesh manipulation ------------------------------------------------------

    /// Transform all vertices, by displacing and rotating (rotation via matrix, so also scaling if needed).
    virtual void Transform(const ChVector3d& displ, const ChMatrix33d& rotscale) override;

    /// Utility function for merging multiple meshes.
    static ChTriangleMeshConnected Merge(std::vector<ChTriangleMeshConnected>& meshes);

    /// Compute and load mass, barycenter, and inertia tensor.
    /// If body_coords=true, inertia is calculated relative to the center of mass.
    /// This function assumes the object has a constant density of 1.
    /// To use a density value `rho`, multiply the output mass by `rho` and the output inertia by `rho`.
    /// The output results are scaled by scale^3 (for the mass) and by scale^5 for inertia.
    void ComputeMassProperties(bool body_coords, double& mass, ChVector3d& center, ChMatrix33d& inertia, double scale = 1.0) const;

    /// Compute and return mass, barycenter, and inertia tensor.
    /// If body_coords=true, the inertia is calculated relative to the center of mass.
    /// This function assumes the object has a constant density of 1.
    /// To use a density value `rho`, multiply the output mass by `rho` and the output inertia by `rho`.
    /// The output results are scaled by scale^3 (for the mass) and by scale^5 for inertia.
    ChMassProperties ComputeMassProperties(bool body_coords, double scale = 1.0) const;

    /// Create a map of neighboring triangles, vector [Ti TieA TieB TieC] (the free sides have triangle id = -1).
    /// Return false if some edge has more than 2 neighboring triangles
    bool ComputeNeighbouringTriangleMap(std::vector<std::array<int, 4>>& tri_map) const;

    /// Create a winged edge structure, map of {key, value} as {{edgevertexA, edgevertexB}, {triangleA, triangleB}}.
    /// If allow_single_wing = false, only edges with at least 2 triangles are returned.
    /// Else, also boundary edges with 1 triangle (the free side has triangle id = -1).
    /// Return false if some edge has more than 2 neighboring triangles.
    bool ComputeWingedEdges(std::map<std::pair<int, int>, std::pair<int, int>>& winged_edges, bool allow_single_wing = true) const;

    /// Connect overlapping vertices.
    /// This can be used to attempt to repair a mesh with 'open edges' to transform it into a watertight mesh.
    /// Say, if a cube is modeled with 6 faces with 4 distinct vertices each, it might display properly, but for
    /// some algorithms (e.g. collision detection) topological information might be needed, hence adjacent faces must be connected.
    /// Return the number of merged vertices.
    int RepairDuplicateVertices(double tolerance = 1e-18  ///< ignore vertices closer than this value
    );

    /// Offset the mesh, by a specified value, orthogonally to the faces.
    /// The offset can be inward or outward.
    /// Note: self-collisions and inverted faces resulting from excessive offsets are NOT trimmed;
    ///       so this is mostly meant to be a fast tool for making small offsets.
    bool MakeOffset(double offset);

    /// Split a given edge by inserting a vertex in the middle: from two triangles one gets four triangles.
    /// It also interpolate normals, colors, uv. It also used and modifies the triangle neighboring map.
    /// If the two triangles do not share an edge, returns false.
    /// The auxiliary buffers are used for interpolation and assumed to be indexed like the vertex buffer.
    bool SplitEdge(int itA,                                              ///< triangle A index,
                   int itB,                                              ///< triangle B index, -1 if not present (free edge on A)
                   int neA,                                              ///< n.edge on tri A: 0,1,2
                   int neB,                                              ///< n.edge on tri B: 0,1,2
                   int& itA_1,                                           ///< returns the index of split triangle A, part1
                   int& itA_2,                                           ///< returns the index of split triangle A, part2
                   int& itB_1,                                           ///< returns the index of split triangle B, part1
                   int& itB_2,                                           ///< returns the index of split triangle B, part2
                   std::vector<std::array<int, 4>>& tri_map,             ///< triangle neighbouring map
                   std::vector<std::vector<double>*>& aux_data_double,   ///< auxiliary buffers
                   std::vector<std::vector<int>*>& aux_data_int,         ///< auxiliary buffers
                   std::vector<std::vector<bool>*>& aux_data_bool,       ///< auxiliary buffers
                   std::vector<std::vector<ChVector3d>*>& aux_data_vect  ///< auxiliary buffers
    );

    /// Callback class to be used optionally in RefineMeshEdges() method.
    class ChRefineEdgeCriterion {
      public:
        virtual ~ChRefineEdgeCriterion() {}

        // Compute length of an edge or more in general a cost function - the higher, the more likely the edge must be cut.
        virtual double ComputeLength(const int vert_a, const int vert_b, ChTriangleMeshConnected* mesh) = 0;
    };

    /// Performs mesh refinement using Rivara LEPP long-edge bisection algorithm.
    /// Given a conforming, non-degenerate triangulation, it construct a locally refined
    /// triangulation with a prescribed resolution. This algorithm, for increasing resolution,
    /// tends to produce triangles with bounded angles even if starting from skewed/skinny triangles in the coarse mesh.
    /// Based on "Multithread parallelization of Lepp-bisection algorithms"
    ///    M.-C. Rivara et al., Applied Numerical Mathematics 62 (2012) 473-488
    /// The auxiliary buffers are used for refinement and assumed to be indexed like the vertex buffer.
    void RefineMeshEdges(std::vector<int>& marked_tris,              ///< indices of triangles to refine (also surrounding triangles might be affected by refinements)
                         double edge_maxlen,                         ///< maximum length of edge (small values give higher resolution)
                         ChRefineEdgeCriterion* criterion,           ///< criterion for computing length (or other cost function) of edge, if null uses default (euclidean length)
                         std::vector<std::array<int, 4>>* atri_map,  ///< optional triangle connectivity map
                         std::vector<std::vector<double>*>& aux_data_double,   ///< auxiliary buffer
                         std::vector<std::vector<int>*>& aux_data_int,         ///< auxiliary buffer
                         std::vector<std::vector<bool>*>& aux_data_bool,       ///< auxiliary buffer
                         std::vector<std::vector<ChVector3d>*>& aux_data_vect  ///< auxiliary buffer
    );

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  public:
    std::vector<ChVector3d> m_vertices;
    std::vector<ChVector3d> m_normals;
    std::vector<ChVector2d> m_UV;
    std::vector<ChColor> m_colors;

    std::vector<ChVector3i> m_face_v_indices;
    std::vector<ChVector3i> m_face_n_indices;
    std::vector<ChVector3i> m_face_uv_indices;
    std::vector<ChVector3i> m_face_col_indices;
    std::vector<int> m_face_mat_indices;

    std::string m_filename;  ///< file string if loading an obj file

    std::vector<ChProperty*> m_properties_per_vertex;
    std::vector<ChProperty*> m_properties_per_face;

    std::vector<ChVector3d> m_tmp_vectors;
    std::vector<ChColor> m_tmp_colors;
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChTriangleMeshConnected, 0)

}  // end namespace chrono

#endif
