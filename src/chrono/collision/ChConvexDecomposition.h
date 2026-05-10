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
// Authors: Alessandro Tasora, Dario Fusai
// =============================================================================

#ifndef CH_CONVEX_DECOMPOSITION_H
#define CH_CONVEX_DECOMPOSITION_H

#include "chrono/core/ChApiCE.h"
#include "chrono/geometry/ChTriangleMesh.h"

#include "chrono_thirdparty/HACD/hacdHACD.h"
#include "chrono_thirdparty/HACDv2/HACD.h"
#include "chrono_thirdparty/VHACD/VHACD.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Base interface class for convex decomposition.
class ChApi ChConvexDecomposition {
  public:
    /// Basic constructor.
    ChConvexDecomposition();

    /// Destructor.
    virtual ~ChConvexDecomposition();

    /// Reset the input mesh data.
    virtual void Reset() = 0;

    /// Add a triangle, by passing three points for vertices (that will be copied, not referenced).
    /// Note: the vertices must have proper winding (oriented triangle, normal pointing outside).
    virtual bool AddTriangle(const ChVector3d& v1, const ChVector3d& v2, const ChVector3d& v3) = 0;

    /// Add a triangle, by passing a ChTriangle object (that will be copied, not referenced).
    /// Note: the vertices must have proper winding (oriented triangle, normal pointing outside).
    virtual bool AddTriangle(const ChTriangle& t1);

    /// Add a triangle mesh, by passing an entire ChTriangleMesh object.
    /// Note: the triangles must define closed volumes (holes, gaps in edges, etc. may trouble the decomposition).
    virtual bool AddTriangleMesh(const ChTriangleMesh& tm);

    /// Perform the convex decomposition.
    virtual unsigned int ComputeConvexDecomposition() = 0;

    /// Get the number of computed hulls after the convex decomposition.
    virtual unsigned int GetHullCount() = 0;

    /// Get the n-th computed convex hull, by filling a ChTriangleMesh object that is passed as a parameter.
    virtual bool GetConvexHullResult(unsigned int hull_index, ChTriangleMesh& convextrimesh) = 0;

    /// Get the n-th computed convex hull, by filling a vector with related vertices.
    virtual bool GetConvexHullResult(unsigned int hull_index, std::vector<ChVector3d>& convexhull) = 0;

    /// Write the convex decomposition to a '.chulls' file, where each hull is a sequence of non-repeated vertices.
    virtual bool WriteConvexHullsAsChullsFile(std::ostream& stream);

    /// Write the convex decomposition to a Wavefront '.obj' file, where each hull is a separate group.
    virtual void WriteConvexHullsAsWavefrontObj(std::ostream& stream) = 0;
};

/// Class for wrapping the HACD convex decomposition code by Khaled Mamou.
class ChApi ChConvexDecompositionHACD : public ChConvexDecomposition {
  public:
    /// Basic constructor.
    ChConvexDecompositionHACD();

    /// Destructor.
    virtual ~ChConvexDecompositionHACD();

    /// Reset the input mesh data.
    virtual void Reset() override;

    /// Add a triangle, by passing three points for vertices.
    /// Note: the vertices must have proper winding (oriented triangle, normal pointing outside).
    virtual bool AddTriangle(const ChVector3d& v1, const ChVector3d& v2, const ChVector3d& v3) override;

    /// Add a triangle mesh soup, by passing an entire ChTriangleMesh object.
    /// Note 1: the triangle mesh does not need connectivity information (a basic 'triangle soup' is enough).
    /// Note 2: all vertices must have proper winding (oriented triangles, normals pointing outside).
    /// Note 3: the triangles must define closed volumes (holes, gaps in edges, etc. may trouble the decomposition).
    virtual bool AddTriangleMesh(const ChTriangleMesh& tm) override;

    /// Set the parameters for this convex decomposition algorithm.
    /// Use this function before calling ComputeConvexDecomposition().
    void SetParameters(unsigned int num_clusters = 2,          ///< minimum number of clusters generated
                       unsigned int target_decimation = 0,     ///< if 0 no decimation, otherwise number of vertices in decimated mesh
                       double small_cluster_threshold = 0.25,  ///< threshold for small cluster grouping, as percentage of tot mesh surface
                       bool add_faces_points = false,          ///< add points in faces in concavity
                       bool add_extra_dist_points = false,     ///< add extra points in concavity
                       double concavity = 100.0,               ///< max allowed concavity
                       double cc_connect_dist = 30,            ///< max allowed distance for cc to be connected
                       double volume_weight = 0.0,             ///< 'beta' parameter, ie. volume weight
                       double compacity_alpha = 0.1,           ///< 'alpha' parameter, ie. compacity weight
                       unsigned int num_vertices_per_CH = 50   ///< max vertices per conxex hull
    );

    /// Perform the convex decomposition.
    /// This operation is time consuming, and it may take a while to complete.
    /// Quality of the results can depend a lot on the parameters.
    /// Also, meshes with triangles that are not well oriented (normals always pointing outside) or with gaps/holes, may give wrong results.
    virtual unsigned int ComputeConvexDecomposition() override;

    /// Get the number of computed hulls after the convex decomposition.
    virtual unsigned int GetHullCount() override;

    /// Get the n-th computed convex hull, by filling a ChTriangleMesh object that is passed as a parameter.
    /// Note 1: passed ChTriangleMesh is cleared before populating it.
    /// Note 2: passed ChTriangleMesh is filled with disconnected triangles.
    virtual bool GetConvexHullResult(unsigned int hullIndex, ChTriangleMesh& convextrimesh) override;

    /// Get the n-th computed convex hull, by filling a vector with related vertices.
    /// Note: passed vector of points is cleared before populating it.
    virtual bool GetConvexHullResult(unsigned int hullIndex, std::vector<ChVector3d>& convexhull) override;

    /// Write the convex decomposition to a Wavefront '.obj' file, where each hull is a separate group.
    virtual void WriteConvexHullsAsWavefrontObj(std::ostream& stream) override;

  private:
    HACD::HACD* m_HACD;
    std::vector<HACD::Vec3<HACD::Real>> m_points;
    std::vector<HACD::Vec3<long>> m_triangles;
};

/// Class for wrapping the HACD convex decomposition code revisited by John Ratcliff.
class ChApi ChConvexDecompositionHACDv2 : public ChConvexDecomposition {
  public:
    /// Basic constructor.
    ChConvexDecompositionHACDv2();

    /// Destructor.
    virtual ~ChConvexDecompositionHACDv2();

    /// Reset the input mesh data.
    virtual void Reset() override;

    /// Add a triangle, by passing three points for vertices.
    /// Note: the vertices must have proper winding (oriented triangle, normal pointing outside).
    virtual bool AddTriangle(const ChVector3d& v1, const ChVector3d& v2, const ChVector3d& v3) override;

    /// Add a triangle mesh soup, by passing an entire ChTriangleMesh object.
    /// Note 1: the triangle mesh does not need connectivity information (a basic 'triangle soup' is enough).
    /// Note 2: all vertices must have proper winding (oriented triangles, normals pointing outside).
    /// Note 3: the triangles must define closed volumes (holes, gaps in edges, etc. may trouble the decomposition).
    virtual bool AddTriangleMesh(const ChTriangleMesh& tm) override;

    /// Set the parameters for this convex decomposition algorithm.
    /// Use this function before calling ComputeConvexDecomposition().
    /// Repeated vertices within \a fuse_tolerance threshold will be fused.
    void SetParameters(unsigned int max_hull_count = 256,
                       unsigned int max_merge_hull_count = 256,
                       unsigned int max_hull_vertices = 64,
                       float concavity = 0.2f,
                       float small_cluster_threshold = 0.0f,
                       float fuse_tolerance = 1e-9f,
                       bool verbose = true);

    /// Perform the convex decomposition.
    /// This operation is time consuming, and it may take a while to complete.
    /// Quality of the results can depend a lot on the parameters.
    /// Also, meshes with triangles that are not well oriented (normals always pointing outside) or with gaps/holes, may give wrong results.
    virtual unsigned int ComputeConvexDecomposition() override;

    /// Get the number of computed hulls after the convex decomposition.
    virtual unsigned int GetHullCount() override;

    /// Get the n-th computed convex hull, by filling a ChTriangleMesh object that is passed as a parameter.
    /// Note 1: passed ChTriangleMesh is cleared before populating it.
    /// Note 2: passed ChTriangleMesh is filled with disconnected triangles.
    virtual bool GetConvexHullResult(unsigned int hullIndex, ChTriangleMesh& convextrimesh) override;

    /// Get the n-th computed convex hull, by filling a vector with related vertices.
    /// Note: passed vector of points is cleared before populating it.
    virtual bool GetConvexHullResult(unsigned int hullIndex, std::vector<ChVector3d>& convexhull) override;

    /// Write the convex decomposition to a Wavefront '.obj' file, where each hull is a separate group.
    virtual void WriteConvexHullsAsWavefrontObj(std::ostream& stream) override;

  private:
    HACD::HACD_API::Desc m_descriptor;
    HACD::HACD_API* m_HACD;
    std::vector<ChVector3d> m_points;
    std::vector<ChVector3i> m_triangles;
    double m_fuse_tol;
    bool m_verbose;
};

/// Class for wrapping the V-HACD convex decomposition code by Khaled Mamou.
class ChApi ChConvexDecompositionVHACD : public ChConvexDecomposition {
  public:
    /// Basic constructor.
    ChConvexDecompositionVHACD();

    /// Destructor.
    virtual ~ChConvexDecompositionVHACD();

    /// Reset the input mesh data.
    virtual void Reset() override;

    /// Add a triangle, by passing three points for vertices.
    /// Note: the vertices must have proper winding (oriented triangle, normal pointing outside).
    virtual bool AddTriangle(const ChVector3d& v1, const ChVector3d& v2, const ChVector3d& v3) override;

    /// Set the parameters for this convex decomposition algorithm.
    /// Use this function before calling ComputeConvexDecomposition().
    void SetParameters(unsigned int max_chull_count = 256,     ///< max number of chulls to produce
                       unsigned int max_verts_per_chull = 64,  ///< max number of vertices per chull
                       unsigned int voxel_resolution = 1000,   ///< voxel resolution to use
                       double min_volume_perc_error = 1.0,     ///< min percentage of voxel volume wrt chull allowed
                       unsigned int max_recursion_depth = 10,  ///< max recursion depth
                       bool shrink_wrap = true                 ///< shrink voxel positions to the source mesh on output
    );

    /// Perform the convex decomposition.
    /// This operation is time consuming, and it may take a while to complete.
    /// Quality of the results can depend a lot on the parameters.
    /// Also, meshes with triangles that are not well oriented (normals always pointing outside) or with gaps/holes, may give wrong results.
    virtual unsigned int ComputeConvexDecomposition() override;

    /// Get the number of computed hulls after the convex decomposition.
    virtual unsigned int GetHullCount() override;

    /// Get the n-th computed convex hull, by filling a ChTriangleMesh object that is passed as a parameter.
    /// Note 1: passed ChTriangleMesh is cleared before populating it.
    /// Note 2: passed ChTriangleMesh is filled with disconnected triangles.
    virtual bool GetConvexHullResult(unsigned int hull_index, ChTriangleMesh& convextrimesh) override;

    /// Get the n-th computed convex hull, by filling a vector with related vertices.
    /// Note: passed vector of points is cleared before populating it.
    virtual bool GetConvexHullResult(unsigned int hull_index, std::vector<ChVector3d>& convexhull) override;

    /// Write the convex decomposition to a Wavefront '.obj' file, where each hull is a separate group.
    virtual void WriteConvexHullsAsWavefrontObj(std::ostream& stream) override;

  private:
    VHACD::IVHACD* m_vhacd;
    VHACD::IVHACD::Parameters m_params;
    std::vector<double> m_points;     ///< flattened vector of vertices xyz components
    std::vector<uint32_t> m_indices;  ///< flattened vector of triangle indices abc components
};

/// @} chrono_collision

}  // end namespace chrono

#endif