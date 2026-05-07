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
#include "chrono_thirdparty/HACDv2/wavefront.h"

namespace chrono {

//
// Utility functions to process bad topology in meshes with repeated vertices
//  (to be optimized for performance; in future it is better to use hash lookup)
//

int GetIndex(ChVector3d vertex, std::vector<ChVector3d>& vertexOUT, double tol) {
    // Suboptimal: search vertices with same position and reuse, (in future: adopt hash map..)
    for (unsigned int iv = 0; iv < vertexOUT.size(); iv++) {
        if (vertex.Equals(vertexOUT[iv], tol)) {
            return iv;
        }
    }
    // not found, so add it to new vertices
    vertexOUT.push_back(vertex);
    return ((int)vertexOUT.size() - 1);
}

void FuseMesh(std::vector<ChVector3d>& vertexIN,
              std::vector<ChVector3i>& triangleIN,
              std::vector<ChVector3d>& vertexOUT,
              std::vector<ChVector3i>& triangleOUT,
              double tol = 0.0) {
    vertexOUT.clear();
    triangleOUT.clear();
    for (unsigned int it = 0; it < triangleIN.size(); it++) {
        unsigned int i1 = GetIndex(vertexIN[triangleIN[it].x()], vertexOUT, tol);
        unsigned int i2 = GetIndex(vertexIN[triangleIN[it].y()], vertexOUT, tol);
        unsigned int i3 = GetIndex(vertexIN[triangleIN[it].z()], vertexOUT, tol);

        ChVector3i merged_triangle(i1, i2, i3);

        triangleOUT.push_back(merged_triangle);
    }
}

////////////////////////////////////////////////////////////////////////////

/// Basic constructor
ChConvexDecomposition::ChConvexDecomposition() {}

/// Destructor
ChConvexDecomposition::~ChConvexDecomposition() {}

bool ChConvexDecomposition::AddTriangle(const ChTriangle& t1) {
    return AddTriangle(t1.p1, t1.p2, t1.p3);  // add the input mesh one triangle at a time.
}

bool ChConvexDecomposition::AddTriangleMesh(const ChTriangleMesh& tm) {
    for (unsigned int i = 0; i < tm.GetNumTriangles(); i++) {
        if (!AddTriangle(tm.GetTriangle(i)))
            return false;
    }
    return true;
}

bool ChConvexDecomposition::WriteConvexHullsAsChullsFile(std::ostream& mstream) {
    mstream << std::setprecision(9) << std::defaultfloat
            << "# Chrono convex hulls decomposition -- .chulls format (list of vertices)\n\n";

    for (unsigned int ih = 0; ih < GetHullCount(); ih++) {
        std::vector<ChVector3d> convexhull;

        if (!GetConvexHullResult(ih, convexhull))
            return false;

        mstream << "hull_" << ih << "\n";
        for (const auto& vert : convexhull) {
            mstream << vert.x() << " " << vert.y() << " " << vert.z() << "\n";
        }
    }
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

//
//  ChConvexDecompositionHACD
//

/// Basic constructor
ChConvexDecompositionHACD::ChConvexDecompositionHACD() {
    myHACD = HACD::CreateHACD();
}

/// Destructor
ChConvexDecompositionHACD::~ChConvexDecompositionHACD() {
    if (myHACD)
        HACD::DestroyHACD(myHACD);
    myHACD = 0;
}

void ChConvexDecompositionHACD::Reset() {
    if (myHACD)
        HACD::DestroyHACD(myHACD);
    myHACD = 0;
    myHACD = HACD::CreateHACD();
    points.clear();
    triangles.clear();
}

bool ChConvexDecompositionHACD::AddTriangle(const ChVector3d& v1, const ChVector3d& v2, const ChVector3d& v3) {
    long lastpoint = (long)points.size();
    HACD::Vec3<HACD::Real> vertex1(v1.x(), v1.y(), v1.z());
    HACD::Vec3<HACD::Real> vertex2(v2.x(), v2.y(), v2.z());
    HACD::Vec3<HACD::Real> vertex3(v3.x(), v3.y(), v3.z());
    points.push_back(vertex1);
    points.push_back(vertex2);
    points.push_back(vertex3);
    HACD::Vec3<long> newtri(lastpoint, lastpoint + 1, lastpoint + 2);
    triangles.push_back(newtri);
    return true;
}

bool ChConvexDecompositionHACD::AddTriangleMesh(const ChTriangleMesh& tm) {
    for (unsigned int i = 0; i < tm.GetNumTriangles(); i++) {
        if (!ChConvexDecomposition::AddTriangle(tm.GetTriangle(i)))
            return false;
    }
    return true;
}

void ChConvexDecompositionHACD::SetParameters(unsigned int nClusters,
                                              unsigned int targetDecimation,
                                              double smallClusterThreshold,
                                              bool addFacesPoints,
                                              bool addExtraDistPoints,
                                              double concavity,
                                              double ccConnectDist,
                                              double volumeWeight,
                                              double compacityAlpha,
                                              unsigned int nVerticesPerCH) {
    myHACD->SetNClusters(nClusters);
    myHACD->SetNTargetTrianglesDecimatedMesh(targetDecimation);
    myHACD->SetSmallClusterThreshold(smallClusterThreshold);
    myHACD->SetAddFacesPoints(addFacesPoints);
    myHACD->SetAddExtraDistPoints(addExtraDistPoints);
    myHACD->SetConcavity(concavity);
    myHACD->SetConnectDist(ccConnectDist);
    myHACD->SetVolumeWeight(volumeWeight);
    myHACD->SetCompacityWeight(compacityAlpha);
    myHACD->SetNVerticesPerCH(nVerticesPerCH);
}

int ChConvexDecompositionHACD::ComputeConvexDecomposition() {
    myHACD->SetPoints(&points[0]);
    myHACD->SetNPoints(points.size());
    myHACD->SetTriangles(&triangles[0]);
    myHACD->SetNTriangles(triangles.size());

    myHACD->Compute();

    return (int)myHACD->GetNClusters();
}

/// Get the number of computed hulls after the convex decomposition
unsigned int ChConvexDecompositionHACD::GetHullCount() {
    return (unsigned int)myHACD->GetNClusters();
}

bool ChConvexDecompositionHACD::GetConvexHullResult(unsigned int hullIndex, std::vector<ChVector3d>& convexhull) {
    if (hullIndex > myHACD->GetNClusters())
        return false;

    size_t nPoints = myHACD->GetNPointsCH(hullIndex);
    size_t nTriangles = myHACD->GetNTrianglesCH(hullIndex);

    std::vector<HACD::Vec3<HACD::Real>> pointsCH(nPoints);
    std::vector<HACD::Vec3<long>> trianglesCH(nTriangles);
    myHACD->GetCH(hullIndex, pointsCH.data(), trianglesCH.data());

    // convert to chrono data
    convexhull.clear();  // clear, for safety
    for (unsigned int i = 0; i < nPoints; i++) {
        ChVector3d point(pointsCH[i].X(), pointsCH[i].Y(), pointsCH[i].Z());
        convexhull.push_back(point);
    }

    return true;
}

/// Get the n-th computed convex hull, by filling a ChTriangleMesh object that is passed as a parameter
bool ChConvexDecompositionHACD::GetConvexHullResult(unsigned int hullIndex, ChTriangleMesh& convextrimesh) {
    if (hullIndex > myHACD->GetNClusters())
        return false;

    size_t nPoints = myHACD->GetNPointsCH(hullIndex);
    size_t nTriangles = myHACD->GetNTrianglesCH(hullIndex);

    std::vector<HACD::Vec3<HACD::Real>> pointsCH(nPoints);
    std::vector<HACD::Vec3<long>> trianglesCH(nTriangles);
    myHACD->GetCH(hullIndex, pointsCH.data(), trianglesCH.data());

    convextrimesh.Clear();  // clear, for safety
    for (unsigned int i = 0; i < nTriangles; i++) {
        unsigned int i1 = trianglesCH[i].X();
        unsigned int i2 = trianglesCH[i].Y();
        unsigned int i3 = trianglesCH[i].Z();
        convextrimesh.AddTriangle(ChVector3d(pointsCH[i1].X(), pointsCH[i1].Y(), pointsCH[i1].Z()),
                                  ChVector3d(pointsCH[i2].X(), pointsCH[i2].Y(), pointsCH[i2].Z()),
                                  ChVector3d(pointsCH[i3].X(), pointsCH[i3].Y(), pointsCH[i3].Z()));
    }

    return true;
}

//
// SERIALIZATION
//

void ChConvexDecompositionHACD::WriteConvexHullsAsWavefrontObj(std::ostream& mstream) {
    mstream << std::setprecision(9) << "# Chrono convex hulls decomposition -- Wavefront .obj format\n\n";

    NxU32 vcount_base = 1;
    NxU32 vcount_total = 0;
    NxU32 tcount_total = 0;
    for (unsigned int hullIndex = 0; hullIndex < this->GetHullCount(); hullIndex++) {
        mstream << "g hull_" << hullIndex << "\n";

        size_t nPoints = myHACD->GetNPointsCH(hullIndex);
        size_t nTriangles = myHACD->GetNTrianglesCH(hullIndex);

        std::vector<HACD::Vec3<HACD::Real>> pointsCH(nPoints);
        std::vector<HACD::Vec3<long>> trianglesCH(nTriangles);
        myHACD->GetCH(hullIndex, pointsCH.data(), trianglesCH.data());

        vcount_total += (NxU32)nPoints;
        tcount_total += (NxU32)nTriangles;
        for (unsigned int i = 0; i < nPoints; i++) {
            mstream << "v " << pointsCH[i].X() << " " << pointsCH[i].Y() << " " << pointsCH[i].Z() << "\n";
        }
        for (unsigned int i = 0; i < nTriangles; i++) {
            unsigned int i1 = trianglesCH[i].X();
            unsigned int i2 = trianglesCH[i].Y();
            unsigned int i3 = trianglesCH[i].Z();
            mstream << "f " << i1 + vcount_base << " " << i2 + vcount_base << " " << i3 + vcount_base << "\n";
        }
        vcount_base += (NxU32)nPoints;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

//
//  ChConvexDecompositionHACDv2
//

/// Basic constructor
ChConvexDecompositionHACDv2::ChConvexDecompositionHACDv2() {
    descriptor.init();
    gHACD = HACD::createHACD_API();
    fuse_tol = 1e-9;
}

/// Destructor
ChConvexDecompositionHACDv2::~ChConvexDecompositionHACDv2() {
    gHACD->release();  // will delete itself
}

void ChConvexDecompositionHACDv2::Reset() {
    descriptor.init();
    gHACD->releaseHACD();

    points.clear();
    triangles.clear();
}

bool ChConvexDecompositionHACDv2::AddTriangle(const ChVector3d& v1, const ChVector3d& v2, const ChVector3d& v3) {
    int lastpoint = static_cast<int>(points.size());
    points.push_back(v1);
    points.push_back(v2);
    points.push_back(v3);
    ChVector3i newtri(lastpoint, lastpoint + 1, lastpoint + 2);
    triangles.push_back(newtri);
    return true;
}

bool ChConvexDecompositionHACDv2::AddTriangleMesh(const ChTriangleMesh& tm) {
    for (unsigned int i = 0; i < tm.GetNumTriangles(); i++) {
        if (!ChConvexDecomposition::AddTriangle(tm.GetTriangle(i)))
            return false;
    }
    return true;
}

void ChConvexDecompositionHACDv2::SetParameters(unsigned int mmMaxHullCount,
                                                unsigned int mmMaxMergeHullCount,
                                                unsigned int mmMaxHullVertices,
                                                float mmConcavity,
                                                float mmSmallClusterThreshold,
                                                float mmFuseTol) {
    // descriptor.init();
    descriptor.mMaxHullCount = mmMaxHullCount;
    descriptor.mMaxMergeHullCount = mmMaxMergeHullCount;
    descriptor.mMaxHullVertices = mmMaxHullVertices;
    descriptor.mConcavity = mmConcavity;
    descriptor.mSmallClusterThreshold = mmSmallClusterThreshold;
    fuse_tol = mmFuseTol;
}

class MyCallback : public hacd::ICallback {
  public:
    virtual bool Cancelled() {
        // Don't have a cancel button in the test console app.
        return false;
    }

    virtual void ReportProgress(const char* message, hacd::HaF32 progress) { std::cout << message; }
};

int ChConvexDecompositionHACDv2::ComputeConvexDecomposition() {
    if (!gHACD)
        return 0;

    // Preprocess: fuse repeated vertices
    std::vector<ChVector3d> points_FUSED;
    std::vector<ChVector3i> triangles_FUSED;
    FuseMesh(points, triangles, points_FUSED, triangles_FUSED, fuse_tol);

    // Convert to HACD format
    descriptor.mTriangleCount = (hacd::HaU32)triangles_FUSED.size();
    descriptor.mVertexCount = (hacd::HaU32)points_FUSED.size();
    descriptor.mIndices = new hacd::HaU32[3 * descriptor.mTriangleCount];
    descriptor.mVertices = new hacd::HaF32[3 * descriptor.mVertexCount];
    for (unsigned int mv = 0; mv < points_FUSED.size(); mv++) {
        ((hacd::HaF32*)descriptor.mVertices)[mv * 3 + 0] = (float)points_FUSED[mv].x();
        ((hacd::HaF32*)descriptor.mVertices)[mv * 3 + 1] = (float)points_FUSED[mv].y();
        ((hacd::HaF32*)descriptor.mVertices)[mv * 3 + 2] = (float)points_FUSED[mv].z();
    }
    for (unsigned int mt = 0; mt < triangles_FUSED.size(); mt++) {
        ((hacd::HaU32*)descriptor.mIndices)[mt * 3 + 0] = triangles_FUSED[mt].x();
        ((hacd::HaU32*)descriptor.mIndices)[mt * 3 + 1] = triangles_FUSED[mt].y();
        ((hacd::HaU32*)descriptor.mIndices)[mt * 3 + 2] = triangles_FUSED[mt].z();
    }

    MyCallback callback;
    descriptor.mCallback = static_cast<hacd::ICallback*>(&callback);

    // Perform the decomposition
    hacd::HaU32 hullCount = gHACD->performHACD(descriptor);

    delete[] descriptor.mIndices;
    delete[] descriptor.mVertices;
    descriptor.mTriangleCount = 0;
    descriptor.mVertexCount = 0;

    return hullCount;
}

/// Get the number of computed hulls after the convex decomposition
unsigned int ChConvexDecompositionHACDv2::GetHullCount() {
    return gHACD->getHullCount();
}

bool ChConvexDecompositionHACDv2::GetConvexHullResult(unsigned int hullIndex, std::vector<ChVector3d>& convexhull) {
    if (hullIndex > gHACD->getHullCount())
        return false;

    const HACD::HACD_API::Hull* hull = gHACD->getHull(hullIndex);
    convexhull.clear();  // clear, for safety
    if (hull) {
        for (hacd::HaU32 i = 0; i < hull->mVertexCount; i++) {
            const hacd::HaF32* p = &hull->mVertices[i * 3];
            convexhull.emplace_back(p[0], p[1], p[2]);
        }
        return true;
    }
    return false;
}

/// Get the n-th computed convex hull, by filling a ChTriangleMesh object that is passed as a parameter
bool ChConvexDecompositionHACDv2::GetConvexHullResult(unsigned int hullIndex, ChTriangleMesh& convextrimesh) {
    if (hullIndex > gHACD->getHullCount())
        return false;

    const HACD::HACD_API::Hull* hull = gHACD->getHull(hullIndex);
    convextrimesh.Clear();  // clear, for safety
    if (hull) {
        for (unsigned int i = 0; i < hull->mTriangleCount; i++) {
            unsigned int i1 = 3 * hull->mIndices[i * 3 + 0];
            unsigned int i2 = 3 * hull->mIndices[i * 3 + 1];
            unsigned int i3 = 3 * hull->mIndices[i * 3 + 2];
            convextrimesh.AddTriangle(
                ChVector3d(hull->mVertices[i1 + 0], hull->mVertices[i1 + 1], hull->mVertices[i1 + 2]),
                ChVector3d(hull->mVertices[i2 + 0], hull->mVertices[i2 + 1], hull->mVertices[i2 + 2]),
                ChVector3d(hull->mVertices[i3 + 0], hull->mVertices[i3 + 1], hull->mVertices[i3 + 2]));
        }
    }
    return true;
}

//
// SERIALIZATION
//

void ChConvexDecompositionHACDv2::WriteConvexHullsAsWavefrontObj(std::ostream& mstream) {
    mstream << std::setprecision(9) << "# Chrono convex hulls decomposition -- Wavefront .obj format\n\n";

    std::vector<hacd::HaU32> baseVertex(GetHullCount());
    hacd::HaU32 vertexCount = 0;
    for (hacd::HaU32 i = 0; i < GetHullCount(); i++) {
        const HACD::HACD_API::Hull* hull = gHACD->getHull(i);
        if (hull) {
            baseVertex[i] = vertexCount;
            for (hacd::HaU32 j = 0; j < hull->mVertexCount; j++) {
                const hacd::HaF32* p = &hull->mVertices[j * 3];
                mstream << "v " << p[0] << " " << p[1] << " " << p[2] << "\n";
            }
            vertexCount += hull->mVertexCount;
        }
    }
    for (hacd::HaU32 i = 0; i < GetHullCount(); i++) {
        const HACD::HACD_API::Hull* hull = gHACD->getHull(i);
        if (hull) {
            hacd::HaU32 startVertex = baseVertex[i];
            for (hacd::HaU32 j = 0; j < hull->mTriangleCount; j++) {
                hacd::HaU32 i1 = hull->mIndices[j * 3 + 0] + startVertex + 1;
                hacd::HaU32 i2 = hull->mIndices[j * 3 + 1] + startVertex + 1;
                hacd::HaU32 i3 = hull->mIndices[j * 3 + 2] + startVertex + 1;
                mstream << "f " << i1 << " " << i2 << " " << i3 << "\n";
            }
        }
    }
}

}  // end namespace chrono
