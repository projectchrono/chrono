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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHC_CONVEXDECOMPOSITION_H
#define CHC_CONVEXDECOMPOSITION_H

#include "chrono/collision/convexdecomposition/HACD/hacdHACD.h"
#include "chrono/collision/convexdecomposition/HACDv2/HACD.h"
#include "chrono/collision/convexdecomposition/JR/NvConvexDecomposition.h"
#include "chrono/core/ChApiCE.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"

namespace chrono {
namespace collision {

///
/// Base interface class for convex decomposition.
/// There are some inherited classes that can be instanced to perform
/// different algorithms of decomposition.
///

class ChApi ChConvexDecomposition {
  public:
    //
    // FUNCTIONS
    //

    /// Basic constructor
    ChConvexDecomposition();

    /// Destructor
    virtual ~ChConvexDecomposition();

    /// Reset the input mesh data
    virtual void Reset(void) = 0;

    /// Add a triangle, by passing three points for vertexes.
    /// Note: the vertexes must be properly ordered (oriented triangle, normal pointing outside)
    virtual bool AddTriangle(const ChVector<>& v1, const ChVector<>& v2, const ChVector<>& v3) = 0;

    /// Add a triangle, by passing a  ChTriangle object (that will be copied, not referenced).
    /// Note: the vertexes must be properly ordered (oriented triangle, normal pointing outside)
    virtual bool AddTriangle(const geometry::ChTriangle& t1);

    /// Add a triangle mesh, by passing an entire ChTriangleMesh object.
    /// Note: the triangles must define closed volumes (holes, gaps in edges, etc. may trouble the decomposition)
    virtual bool AddTriangleMesh(const geometry::ChTriangleMesh& tm);

    /// Perform the convex decomposition.
    virtual int ComputeConvexDecomposition() = 0;

    /// Get the number of computed hulls after the convex decomposition
    virtual unsigned int GetHullCount() = 0;

    /// Get the n-th computed convex hull, by filling a ChTriangleMesh object
    /// that is passed as a parameter.
    virtual bool GetConvexHullResult(unsigned int hullIndex, geometry::ChTriangleMesh& convextrimesh) = 0;

    /// Get the n-th computed convex hull, by filling a vector of points of the vertexes of the n-th hull
    /// that is passed as a parameter.
    virtual bool GetConvexHullResult(unsigned int hullIndex, std::vector<ChVector<double> >& convexhull) = 0;

    //
    // SERIALIZATION
    //

    /// Write the convex decomposition to a ".chulls" file,
    /// where each hull is a sequence of x y z coords. Can throw exceptions.
    virtual bool WriteConvexHullsAsChullsFile(ChStreamOutAscii& mstream);

    /// Save the computed convex hulls as a Wavefront file using the
    /// '.obj' fileformat, with each hull as a separate group.
    /// May throw exceptions if file locked etc.
    virtual void WriteConvexHullsAsWavefrontObj(ChStreamOutAscii& mstream) = 0;

    //
    // DATA
    //

  private:
};

///
/// Class for wrapping the HACD convex decomposition code
/// by Khaled Mamou (in the convexdecomposition/ directory)
/// so that it is easier to use it by passing the Chrono
/// structures of type ChTriangleMesh, ChTriangle, etc.
///

class ChApi ChConvexDecompositionHACD : public ChConvexDecomposition {
  public:
    //
    // FUNCTIONS
    //

    /// Basic constructor
    ChConvexDecompositionHACD();

    /// Destructor
    virtual ~ChConvexDecompositionHACD();

    /// Reset the input mesh data
    virtual void Reset(void);

    /// Add a triangle, by passing three points for vertexes.
    /// Note: the vertexes must be properly ordered (oriented triangle, normal pointing outside)
    virtual bool AddTriangle(const ChVector<>& v1, const ChVector<>& v2, const ChVector<>& v3);

    /// Add a triangle, by passing a  ChTriangle object (that will be copied, not referenced).
    /// Note: the vertexes must be properly ordered (oriented triangle, normal pointing outside)
    // virtual bool AddTriangle(const ChTriangle& t1);

    /// Add a triangle mesh soup, by passing an entire ChTriangleMesh object.
    /// Note 1: the triangle mesh does not need connectivity information (a basic 'triangle soup' is enough)
    /// Note 2: all vertexes must be properly ordered (oriented triangles, normals pointing outside).
    /// Note 3: the triangles must define closed volumes (holes, gaps in edges, etc. may trouble the decomposition)
    virtual bool AddTriangleMesh(const geometry::ChTriangleMesh& tm);

    /// Set the parameters for this convex decomposition algorithm.
    /// Use this function before calling ComputeConvexDecomposition().
    void SetParameters(
        unsigned int nClusters = 2,           ///< Minimum number of clusters
        unsigned int targetDecimation = 0,    ///< If 0 no decimation, otherwise n.of vertexes in decimated mesh
        double smallClusterThreshold = 0.25,  ///< Threshold for small cluster grouping
        bool addFacesPoints = false,          ///< Add points in faces in concavity
        bool addExtraDistPoints = false,      ///< Add extra points in concavity
        double concavity = 100.0,             ///< Max allowed concavity
        double ccConnectDist = 30,            ///< Max allowed distance for cc to be connected
        double volumeWeight = 0.0,            ///< 'beta' parameter, ie. volume weight
        double compacityAlpha = 0.1,          ///< 'alpha' paramater
        unsigned int nVerticesPerCH = 50      ///< Max vertices for cc.
    );

    /// Perform the convex decomposition.
    /// This operation is time consuming, and it may take a while to complete.
    /// Quality of the results can depend a lot on the parameters. Also, meshes
    /// with triangles that are not well oriented (normals always pointing outside)
    /// or with gaps/holes, may give wrong results.
    virtual int ComputeConvexDecomposition();

    /// Get the number of computed hulls after the convex decomposition
    virtual unsigned int GetHullCount();

    /// Get the n-th computed convex hull, by filling a ChTriangleMesh object
    /// that is passed as a parameter.
    virtual bool GetConvexHullResult(unsigned int hullIndex, geometry::ChTriangleMesh& convextrimesh);

    /// Get the n-th computed convex hull, by filling a vector of points of the vertexes of the n-th hull
    /// that is passed as a parameter.
    virtual bool GetConvexHullResult(unsigned int hullIndex, std::vector<ChVector<double> >& convexhull);

    //
    // SERIALIZATION
    //

    /// Save the computed convex hulls as a Wavefront file using the
    /// '.obj' fileformat, with each hull as a separate group.
    /// May throw exceptions if file locked etc.
    virtual void WriteConvexHullsAsWavefrontObj(ChStreamOutAscii& mstream);

    //
    // DATA
    //

  private:
    HACD::HACD* myHACD;
    std::vector<HACD::Vec3<HACD::Real> > points;
    std::vector<HACD::Vec3<long> > triangles;
};

///
/// Class for wrapping the NvConvexDecomposition code
/// by John W. Ratcliff.
///

class ChApi ChConvexDecompositionJR : public ChConvexDecomposition {
  public:
    //
    // FUNCTIONS
    //

    /// Basic constructor
    ChConvexDecompositionJR();

    /// Destructor
    virtual ~ChConvexDecompositionJR();

    /// Access directly the wrapped J.W.Ratcliff convex decomposition object
    /// although it shouldn't be necessary, since this class already provide
    /// wrapping to all the functions).
    // CONVEX_DECOMPOSITION::iConvexDecomposition* GetDecompositionObject();

    /// Reset the input mesh data
    virtual void Reset(void);

    /// Add a triangle, by passing three points for vertexes.
    /// Note: the vertexes must be properly ordered (oriented triangle, normal pointing outside)
    virtual bool AddTriangle(const ChVector<>& v1, const ChVector<>& v2, const ChVector<>& v3);

    /// Add a triangle mesh soup, by passing an entire ChTriangleMesh object.
    /// Note 1: the triangle mesh does not need connectivity information (a basic 'triangle soup' is enough)
    /// Note 2: all vertexes must be properly ordered (oriented triangles, normals pointing outside).
    /// Note 3: the triangles must define closed volumes (holes, gaps in edges, etc. may trouble the decomposition)
    virtual bool AddTriangleMesh(const geometry::ChTriangleMesh& tm);

    /// Set the parameters for this convex decomposition algorithm.
    /// Use this function before calling ComputeConvexDecomposition().
    void SetParameters(
        float mskinWidth,                    ///< Skin width on the convex hulls generated
        unsigned int mdecompositionDepth,    ///< Recursion depth for convex decomposition.
        unsigned int mmaxHullVertices,       ///< Maximum number of vertices in output convex hulls.
        float mconcavityThresholdPercent,    ///< The percentage of concavity allowed without causing a split to occur.
        float mmergeThresholdPercent,        ///< The percentage of volume difference allowed to merge two convex hulls.
        float mvolumeSplitThresholdPercent,  ///< The percentage of the total volume of the object above which splits
        /// will still occur.
        bool museInitialIslandGeneration,  ///< Whether or not to perform initial island generation on the input mesh.
        bool museIslandGeneration  ///< Whether or not to perform island generation at each split.  Currently disabled.
    );

    /// Perform the convex decomposition.
    /// This operation is time consuming, and it may take a while to complete.
    /// Quality of the results can depend a lot on the parameters. Also, meshes
    /// with triangles that are not well oriented (normals always pointing outside)
    /// or with gaps/holes, may give wrong results.
    virtual int ComputeConvexDecomposition();

    /// Get the number of computed hulls after the convex decomposition
    virtual unsigned int GetHullCount();

    /// Get the n-th computed convex hull, by filling a ChTriangleMesh object
    /// that is passed as a parameter.
    virtual bool GetConvexHullResult(unsigned int hullIndex, geometry::ChTriangleMesh& convextrimesh);

    /// Get the n-th computed convex hull, by filling a vector of points of the vertexes of the n-th hull
    /// that is passed as a parameter.
    virtual bool GetConvexHullResult(unsigned int hullIndex, std::vector<ChVector<double> >& convexhull);

    //
    // SERIALIZATION
    //

    /// Save the computed convex hulls as a Wavefront file using the
    /// '.obj' fileformat, with each hull as a separate group.
    /// May throw exceptions if file locked etc.
    virtual void WriteConvexHullsAsWavefrontObj(ChStreamOutAscii& mstream);

    //
    // DATA
    //

  private:
    CONVEX_DECOMPOSITION::iConvexDecomposition* mydecomposition;

    float skinWidth;
    unsigned int decompositionDepth;
    unsigned int maxHullVertices;
    float concavityThresholdPercent;
    float mergeThresholdPercent;
    float volumeSplitThresholdPercent;
    bool useInitialIslandGeneration;
    bool useIslandGeneration;
};

///
/// Class for wrapping the HACD convex decomposition code
/// revisited by John Ratcliff
///

class ChApi ChConvexDecompositionHACDv2 : public ChConvexDecomposition {
  public:
    //
    // FUNCTIONS
    //

    /// Basic constructor
    ChConvexDecompositionHACDv2();

    /// Destructor
    virtual ~ChConvexDecompositionHACDv2();

    /// Reset the input mesh data
    virtual void Reset(void);

    /// Add a triangle, by passing three points for vertexes.
    /// Note: the vertexes must be properly ordered (oriented triangle, normal pointing outside)
    virtual bool AddTriangle(const ChVector<>& v1, const ChVector<>& v2, const ChVector<>& v3);

    /// Add a triangle mesh soup, by passing an entire ChTriangleMesh object.
    /// Note 1: the triangle mesh does not need connectivity information (a basic 'triangle soup' is enough)
    /// Note 2: all vertexes must be properly ordered (oriented triangles, normals pointing outside).
    /// Note 3: the triangles must define closed volumes (holes, gaps in edges, etc. may trouble the decomposition)
    virtual bool AddTriangleMesh(const geometry::ChTriangleMesh& tm);

    /// Set the parameters for this convex decomposition algorithm.
    /// Use this function before calling ComputeConvexDecomposition().
    /// Repeated vertices will be fused with \a mFuseTolerance tolerance.
    void SetParameters(unsigned int mMaxHullCount = 256,
                       unsigned int mMaxMergeHullCount = 256,
                       unsigned int mMaxHullVertices = 64,
                       float mConcavity = 0.2f,
                       float mSmallClusterThreshold = 0.0f,
                       float mFuseTolerance = 1e-9);

    /// Perform the convex decomposition.
    /// This operation is time consuming, and it may take a while to complete.
    /// Quality of the results can depend a lot on the parameters. Also, meshes
    /// with triangles that are not well oriented (normals always pointing outside)
    /// or with gaps/holes, may give wrong results.
    virtual int ComputeConvexDecomposition();

    /// Get the number of computed hulls after the convex decomposition
    virtual unsigned int GetHullCount();

    /// Get the n-th computed convex hull, by filling a ChTriangleMesh object
    /// that is passed as a parameter.
    virtual bool GetConvexHullResult(unsigned int hullIndex, geometry::ChTriangleMesh& convextrimesh);

    /// Get the n-th computed convex hull, by filling a vector of points of the vertexes of the n-th hull
    /// that is passed as a parameter.
    virtual bool GetConvexHullResult(unsigned int hullIndex, std::vector<ChVector<double> >& convexhull);

    //
    // SERIALIZATION
    //

    /// Save the computed convex hulls as a Wavefront file using the
    /// '.obj' fileformat, with each hull as a separate group.
    /// May throw exceptions if file locked etc.
    virtual void WriteConvexHullsAsWavefrontObj(ChStreamOutAscii& mstream);

    //
    // DATA
    //

  private:
    HACD::HACD_API::Desc descriptor;
    HACD::HACD_API* gHACD;
    std::vector<ChVector<double> > points;
    std::vector<ChVector<int> > triangles;
    double fuse_tol;
};

}  // end namespace collision
}  // end namespace chrono

#endif