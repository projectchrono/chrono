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

#include <iomanip>
#include "chrono/collision/ChConvexDecomposition.h"

namespace chrono {

// -----------------------------------------------------------------------------
// Utility functions to process bad topology in meshes with repeated vertices
// -----------------------------------------------------------------------------
// TODO: optimize for performance; consider using hash lookup

static unsigned int GetIndex(const ChVector3d& vertex, std::vector<ChVector3d>& vertexOUT, double tol) {
    // Suboptimal: search vertices with same position and reuse, (in future: adopt hash map..)
    for (unsigned int iv = 0; iv < vertexOUT.size(); iv++) {
        if (vertex.Equals(vertexOUT[iv], tol)) {
            return iv;
        }
    }
    // Not found, so add it to new vertices
    vertexOUT.push_back(vertex);
    return static_cast<unsigned int>(vertexOUT.size() - 1);
}

static void FuseMesh(std::vector<ChVector3d>& verticesIN,
                     std::vector<ChVector3i>& trianglesIN,
                     std::vector<ChVector3d>& verticesOUT,
                     std::vector<ChVector3i>& trianglesOUT,
                     double tol) {
    verticesOUT.clear();
    trianglesOUT.clear();
    for (unsigned int it = 0; it < trianglesIN.size(); it++) {
        unsigned int i1 = GetIndex(verticesIN[trianglesIN[it].x()], verticesOUT, tol);
        unsigned int i2 = GetIndex(verticesIN[trianglesIN[it].y()], verticesOUT, tol);
        unsigned int i3 = GetIndex(verticesIN[trianglesIN[it].z()], verticesOUT, tol);

        ChVector3i merged_triangle(i1, i2, i3);

        trianglesOUT.push_back(merged_triangle);
    }
}

// -----------------------------------------------------------------------------
// ChConvexDecomposition (base class)
// -----------------------------------------------------------------------------

ChConvexDecomposition::ChConvexDecomposition() {}

ChConvexDecomposition::~ChConvexDecomposition() {}

bool ChConvexDecomposition::AddTriangle(const ChTriangle& t1) {
    return AddTriangle(t1.p1, t1.p2, t1.p3);
}

bool ChConvexDecomposition::AddTriangleMesh(const ChTriangleMesh& tm) {
    for (unsigned int i = 0; i < tm.GetNumTriangles(); i++) {
        if (!AddTriangle(tm.GetTriangle(i)))
            return false;
    }
    return true;
}

bool ChConvexDecomposition::WriteConvexHullsAsChullsFile(std::ostream& stream) {
    stream << std::setprecision(9) << std::defaultfloat << "# Chrono convex hulls decomposition -- .chulls format (list of vertices)\n\n";

    for (unsigned int ih = 0; ih < GetHullCount(); ih++) {
        std::vector<ChVector3d> convexhull;
        if (!GetConvexHullResult(ih, convexhull))
            return false;

        stream << "hull_" << ih << "\n";
        for (const auto& vert : convexhull) {
            stream << vert.x() << " " << vert.y() << " " << vert.z() << "\n";
        }
        stream << "\n";
    }
    return true;
}

// -----------------------------------------------------------------------------
// ChConvexDecompositionHACDv2
// -----------------------------------------------------------------------------

ChConvexDecompositionHACDv2::ChConvexDecompositionHACDv2() {
    m_descriptor.init();
    m_HACD = HACD::createHACD_API();
    m_fuse_tol = 1e-9;
    m_verbose = true;
}

ChConvexDecompositionHACDv2::~ChConvexDecompositionHACDv2() {
    m_HACD->release();
}

void ChConvexDecompositionHACDv2::Reset() {
    m_descriptor.init();
    m_HACD->releaseHACD();
    m_points.clear();
    m_triangles.clear();
}

bool ChConvexDecompositionHACDv2::AddTriangle(const ChVector3d& v1, const ChVector3d& v2, const ChVector3d& v3) {
    int lastpoint = static_cast<int>(m_points.size());
    m_points.push_back(v1);
    m_points.push_back(v2);
    m_points.push_back(v3);
    ChVector3i newtri(lastpoint, lastpoint + 1, lastpoint + 2);
    m_triangles.push_back(newtri);
    return true;
}

bool ChConvexDecompositionHACDv2::AddTriangleMesh(const ChTriangleMesh& tm) {
    for (unsigned int i = 0; i < tm.GetNumTriangles(); i++) {
        if (!ChConvexDecomposition::AddTriangle(tm.GetTriangle(i)))
            return false;
    }
    return true;
}

void ChConvexDecompositionHACDv2::SetParameters(unsigned int max_hull_count,
                                                unsigned int max_merge_hull_count,
                                                unsigned int max_hull_vertices,
                                                float concavity,
                                                float small_cluster_threshold,
                                                float fuse_tolerance,
                                                bool verbose) {
    m_descriptor.mMaxHullCount = max_hull_count;
    m_descriptor.mMaxMergeHullCount = max_merge_hull_count;
    m_descriptor.mMaxHullVertices = max_hull_vertices;
    m_descriptor.mConcavity = concavity;
    m_descriptor.mSmallClusterThreshold = small_cluster_threshold;
    m_fuse_tol = fuse_tolerance;
    m_verbose = verbose;
}

class MyCallback : public hacd::ICallback {
  public:
    MyCallback(bool verbose) : m_verbose(verbose) {}

    virtual bool Cancelled() override {
        // Don't have a cancel button in the test console app.
        return false;
    }

    virtual void ReportProgress(const char* message, hacd::HaF32 progress) override {
        if (m_verbose)
            std::cout << message;
    }

  private:
    bool m_verbose = false;
};

unsigned int ChConvexDecompositionHACDv2::ComputeConvexDecomposition() {
    if (!m_HACD)
        return 0;

    // Preprocess: fuse repeated vertices
    std::vector<ChVector3d> points_FUSED;
    std::vector<ChVector3i> triangles_FUSED;
    FuseMesh(m_points, m_triangles, points_FUSED, triangles_FUSED, m_fuse_tol);

    // Convert to HACDv2 format
    std::vector<hacd::HaF32> vertices_buffer(3 * points_FUSED.size());
    for (unsigned int mv = 0; mv < points_FUSED.size(); mv++) {
        vertices_buffer[mv * 3 + 0] = static_cast<hacd::HaF32>(points_FUSED[mv].x());
        vertices_buffer[mv * 3 + 1] = static_cast<hacd::HaF32>(points_FUSED[mv].y());
        vertices_buffer[mv * 3 + 2] = static_cast<hacd::HaF32>(points_FUSED[mv].z());
    }
    std::vector<hacd::HaU32> indices_buffer(3 * triangles_FUSED.size());
    for (unsigned int mt = 0; mt < triangles_FUSED.size(); mt++) {
        indices_buffer[mt * 3 + 0] = static_cast<hacd::HaU32>(triangles_FUSED[mt].x());
        indices_buffer[mt * 3 + 1] = static_cast<hacd::HaU32>(triangles_FUSED[mt].y());
        indices_buffer[mt * 3 + 2] = static_cast<hacd::HaU32>(triangles_FUSED[mt].z());
    }

    m_descriptor.mTriangleCount = static_cast<hacd::HaU32>(triangles_FUSED.size());
    m_descriptor.mVertexCount = static_cast<hacd::HaU32>(points_FUSED.size());
    m_descriptor.mVertices = vertices_buffer.data();
    m_descriptor.mIndices = indices_buffer.data();

    MyCallback callback(m_verbose);
    m_descriptor.mCallback = static_cast<hacd::ICallback*>(&callback);

    // Perform the decomposition
    hacd::HaU32 hullCount = m_HACD->performHACD(m_descriptor);

    // Clear
    m_descriptor.mIndices = nullptr;
    m_descriptor.mVertices = nullptr;
    m_descriptor.mTriangleCount = 0;
    m_descriptor.mVertexCount = 0;

    return static_cast<unsigned int>(hullCount);
}

unsigned int ChConvexDecompositionHACDv2::GetHullCount() {
    return m_HACD->getHullCount();
}

bool ChConvexDecompositionHACDv2::GetConvexHullResult(unsigned int hull_index, std::vector<ChVector3d>& convexhull) {
    if (hull_index >= m_HACD->getHullCount())
        return false;

    const HACD::HACD_API::Hull* hull = m_HACD->getHull(hull_index);
    if (!hull)
        return false;
    convexhull.clear();  // clear, for safety
    convexhull.reserve(hull->mVertexCount);

    for (hacd::HaU32 i = 0; i < hull->mVertexCount; i++) {
        const hacd::HaF32* vert = &hull->mVertices[i * 3];
        convexhull.emplace_back(vert[0], vert[1], vert[2]);
    }

    return true;
}

bool ChConvexDecompositionHACDv2::GetConvexHullResult(unsigned int hull_index, ChTriangleMesh& convextrimesh) {
    if (hull_index >= m_HACD->getHullCount())
        return false;

    const HACD::HACD_API::Hull* hull = m_HACD->getHull(hull_index);
    if (!hull)
        return false;
    convextrimesh.Clear();  // clear, for safety

    for (unsigned int i = 0; i < hull->mTriangleCount; i++) {
        unsigned int i1 = 3 * hull->mIndices[i * 3 + 0];
        unsigned int i2 = 3 * hull->mIndices[i * 3 + 1];
        unsigned int i3 = 3 * hull->mIndices[i * 3 + 2];
        convextrimesh.AddTriangle(ChVector3d(hull->mVertices[i1 + 0], hull->mVertices[i1 + 1], hull->mVertices[i1 + 2]),   //
                                  ChVector3d(hull->mVertices[i2 + 0], hull->mVertices[i2 + 1], hull->mVertices[i2 + 2]),   //
                                  ChVector3d(hull->mVertices[i3 + 0], hull->mVertices[i3 + 1], hull->mVertices[i3 + 2]));  //
    }

    return true;
}

void ChConvexDecompositionHACDv2::WriteConvexHullsAsWavefrontObj(std::ostream& stream) {
    stream << std::setprecision(9) << "# Chrono convex hulls decomposition -- Wavefront .obj format\n\n";

    std::vector<hacd::HaU32> base_vertex(GetHullCount());
    hacd::HaU32 vertex_count = 0;
    for (hacd::HaU32 hull_idx = 0; hull_idx < GetHullCount(); ++hull_idx) {
        const HACD::HACD_API::Hull* hull = m_HACD->getHull(hull_idx);
        if (hull) {
            stream << "g hull_" << hull_idx << "\n";

            // Vertices
            base_vertex[hull_idx] = vertex_count;
            for (hacd::HaU32 j = 0; j < hull->mVertexCount; j++) {
                const hacd::HaF32* p = &hull->mVertices[j * 3];
                stream << "v " << p[0] << " " << p[1] << " " << p[2] << "\n";
            }

            // Triangles
            for (hacd::HaU32 j = 0; j < hull->mTriangleCount; j++) {
                hacd::HaU32 i1 = hull->mIndices[j * 3 + 0] + vertex_count + 1;
                hacd::HaU32 i2 = hull->mIndices[j * 3 + 1] + vertex_count + 1;
                hacd::HaU32 i3 = hull->mIndices[j * 3 + 2] + vertex_count + 1;
                stream << "f " << i1 << " " << i2 << " " << i3 << "\n";
            }

            vertex_count += hull->mVertexCount;
        }
        stream << "\n";
    }
}

// -----------------------------------------------------------------------------
// ChConvexDecompositionVHACD
// -----------------------------------------------------------------------------

ChConvexDecompositionVHACD::ChConvexDecompositionVHACD() {
    // Create VHACD interface
    m_vhacd = VHACD::CreateVHACD();

    // Default settings
    m_params.m_maxConvexHulls = 512;
    m_params.m_maxNumVerticesPerCH = 64;
    m_params.m_resolution = 1000;
    m_params.m_minimumVolumePercentErrorAllowed = 1.0;
    m_params.m_shrinkWrap = true;
    m_params.m_fillMode = VHACD::FillMode::FLOOD_FILL;
}

ChConvexDecompositionVHACD::~ChConvexDecompositionVHACD() {
    m_vhacd->Clean();
    m_vhacd->Release();
}

void ChConvexDecompositionVHACD::Reset() {
    m_vhacd->Clean();
    m_points.clear();
    m_indices.clear();
}

bool ChConvexDecompositionVHACD::AddTriangle(const ChVector3d& v1, const ChVector3d& v2, const ChVector3d& v3) {
    uint32_t lastpoint = static_cast<uint32_t>(m_points.size() / 3);
    m_points.push_back(v1.x());
    m_points.push_back(v1.y());
    m_points.push_back(v1.z());
    //
    m_points.push_back(v2.x());
    m_points.push_back(v2.y());
    m_points.push_back(v2.z());
    //
    m_points.push_back(v3.x());
    m_points.push_back(v3.y());
    m_points.push_back(v3.z());

    m_indices.push_back(lastpoint);
    m_indices.push_back(lastpoint + 1);
    m_indices.push_back(lastpoint + 2);

    return true;
}

void ChConvexDecompositionVHACD::SetParameters(unsigned int max_chull_count,
                                               unsigned int max_verts_per_chull,
                                               unsigned int voxel_resolution,
                                               double min_volume_perc_error,
                                               unsigned int max_recursion_depth,
                                               bool shrink_wrap) {
    m_params.m_maxConvexHulls = max_chull_count;
    m_params.m_maxNumVerticesPerCH = max_verts_per_chull;
    m_params.m_resolution = voxel_resolution;
    m_params.m_minimumVolumePercentErrorAllowed = min_volume_perc_error;
    m_params.m_maxRecursionDepth = max_recursion_depth;
    m_params.m_shrinkWrap = shrink_wrap;
    m_params.m_fillMode = VHACD::FillMode::FLOOD_FILL;
}

unsigned int ChConvexDecompositionVHACD::ComputeConvexDecomposition() {
    if (m_points.empty() || m_indices.empty())
        return 0;

    // TODO: preprocess to remove repeated vertices (?)

    bool success = m_vhacd->Compute(m_points.data(), static_cast<uint32_t>(m_points.size() / 3), m_indices.data(), static_cast<uint32_t>(m_indices.size() / 3), m_params);

    int hull_count = (success) ? m_vhacd->GetNConvexHulls() : 0.0;
    return hull_count;
}

unsigned int ChConvexDecompositionVHACD::GetHullCount() {
    return m_vhacd->GetNConvexHulls();
}

bool ChConvexDecompositionVHACD::GetConvexHullResult(unsigned int hull_index, ChTriangleMesh& convextrimesh) {
    if (hull_index >= m_vhacd->GetNConvexHulls())
        return false;

    VHACD::IVHACD::ConvexHull hull;
    bool success = m_vhacd->GetConvexHull(hull_index, hull);
    if (!success)
        return false;

    convextrimesh.Clear();
    for (const auto& tri : hull.m_triangles) {
        ChVector3d vA(hull.m_points[tri.mI0].mX, hull.m_points[tri.mI0].mY, hull.m_points[tri.mI0].mZ);
        ChVector3d vB(hull.m_points[tri.mI1].mX, hull.m_points[tri.mI1].mY, hull.m_points[tri.mI1].mZ);
        ChVector3d vC(hull.m_points[tri.mI2].mX, hull.m_points[tri.mI2].mY, hull.m_points[tri.mI2].mZ);
        convextrimesh.AddTriangle(vA, vB, vC);
    }

    return true;
}

bool ChConvexDecompositionVHACD::GetConvexHullResult(unsigned int hull_index, std::vector<ChVector3d>& convexhull) {
    if (hull_index >= m_vhacd->GetNConvexHulls())
        return false;

    VHACD::IVHACD::ConvexHull hull;
    bool success = m_vhacd->GetConvexHull(hull_index, hull);
    if (!success)
        return false;

    convexhull.clear(); // clear, for safety
    convexhull.reserve(3 * hull.m_triangles.size());
    for (const auto& tri : hull.m_triangles) {
        convexhull.emplace_back(hull.m_points[tri.mI0].mX, hull.m_points[tri.mI0].mY, hull.m_points[tri.mI0].mZ);
        convexhull.emplace_back(hull.m_points[tri.mI1].mX, hull.m_points[tri.mI1].mY, hull.m_points[tri.mI1].mZ);
        convexhull.emplace_back(hull.m_points[tri.mI2].mX, hull.m_points[tri.mI2].mY, hull.m_points[tri.mI2].mZ);
    }

    return true;
}

void ChConvexDecompositionVHACD::WriteConvexHullsAsWavefrontObj(std::ostream& stream) {
    stream << std::setprecision(9) << "# Chrono convex hulls decomposition -- Wavefront .obj format\n\n";

    uint32_t offset = 0;
    for (unsigned int hull_index = 0; hull_index < GetHullCount(); ++hull_index) {
        VHACD::IVHACD::ConvexHull hull;
        m_vhacd->GetConvexHull(hull_index, hull);
        stream << "g hull_" << hull_index << "\n";

        // Vertices
        for (const auto& vert : hull.m_points) {
            stream << "v " << vert.mX << " " << vert.mY << " " << vert.mZ << "\n";
        }
        // Triangles
        for (const auto& tri : hull.m_triangles) {
            stream << "f " << tri.mI0 + offset + 1 << " " << tri.mI1 + offset + 1 << " " << tri.mI2 + offset + 1 << "\n";
        }
        offset += static_cast<uint32_t>(hull.m_points.size());
        stream << "\n";
    }
}

}  // end namespace chrono
