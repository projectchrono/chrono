//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "chrono/collision/ChCConvexDecomposition.h"
#include "chrono/collision/convexdecomposition/HACDv2/wavefront.h"

namespace chrono {
namespace collision {


//
// Utility functions to process bad topology in meshes with repeated vertices
//  (to be optimized for performance; in future it is better to use hash lookup)
//

int GetIndex(ChVector<double> vertex, std::vector<ChVector<double> >& vertexOUT, double tol) {
    // Suboptimal: search vertexes with same position and reuse, (in future: adopt hash map..)
    for (unsigned int iv = 0; iv < vertexOUT.size(); iv++) {
        if (vertex.Equals(vertexOUT[iv], tol)) {
            return iv;
        }
    }
    // not found, so add it to new vertexes
    vertexOUT.push_back(vertex);
    return ((int)vertexOUT.size() - 1);
}

void FuseMesh(std::vector<ChVector<double> >& vertexIN,
              std::vector<ChVector<int> >& triangleIN,
              std::vector<ChVector<double> >& vertexOUT,
              std::vector<ChVector<int> >& triangleOUT,
              double tol = 0.0) {
    vertexOUT.clear();
    triangleOUT.clear();
    for (unsigned int it = 0; it < triangleIN.size(); it++) {
        unsigned int i1 = GetIndex(vertexIN[triangleIN[it].x()], vertexOUT, tol);
        unsigned int i2 = GetIndex(vertexIN[triangleIN[it].y()], vertexOUT, tol);
        unsigned int i3 = GetIndex(vertexIN[triangleIN[it].z()], vertexOUT, tol);

        ChVector<int> merged_triangle(i1, i2, i3);

        triangleOUT.push_back(merged_triangle);
    }
}

////////////////////////////////////////////////////////////////////////////

/// Basic constructor
ChConvexDecomposition::ChConvexDecomposition() {
}

/// Destructor
ChConvexDecomposition::~ChConvexDecomposition() {
}

bool ChConvexDecomposition::AddTriangle(const geometry::ChTriangle& t1) {
    return this->AddTriangle(t1.p1, t1.p2, t1.p3);  // add the input mesh one triangle at a time.
}

bool ChConvexDecomposition::AddTriangleMesh(const geometry::ChTriangleMesh& tm) {
    for (int i = 0; i < tm.getNumTriangles(); i++) {
        if (!this->AddTriangle(tm.getTriangle(i)))
            return false;
    }
    return true;
}

bool ChConvexDecomposition::WriteConvexHullsAsChullsFile(ChStreamOutAscii& mstream) {
    mstream.SetNumFormat("%0.9f");
    mstream << "# Convex hulls obtained with Chrono::Engine \n# convex decomposition (.chulls format: only vertexes)\n";

    for (unsigned int ih = 0; ih < this->GetHullCount(); ih++) {
        std::vector<ChVector<double> > aconvexhull;

        if (!this->GetConvexHullResult(ih, aconvexhull))
            return false;

        mstream << "hull\n";
        for (unsigned int i = 0; i < aconvexhull.size(); i++) {
            mstream << aconvexhull[i].x() << " " << aconvexhull[i].y() << " " << aconvexhull[i].z() << "\n";
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

void ChConvexDecompositionHACD::Reset(void) {
    if (myHACD)
        HACD::DestroyHACD(myHACD);
    myHACD = 0;
    myHACD = HACD::CreateHACD();
    this->points.clear();
    this->triangles.clear();
}

bool ChConvexDecompositionHACD::AddTriangle(const ChVector<>& v1, const ChVector<>& v2, const ChVector<>& v3) {
    long lastpoint = (long)this->points.size();
    HACD::Vec3<HACD::Real> vertex1(v1.x(), v1.y(), v1.z());
    HACD::Vec3<HACD::Real> vertex2(v2.x(), v2.y(), v2.z());
    HACD::Vec3<HACD::Real> vertex3(v3.x(), v3.y(), v3.z());
    this->points.push_back(vertex1);
    this->points.push_back(vertex2);
    this->points.push_back(vertex3);
    HACD::Vec3<long> newtri(lastpoint, lastpoint + 1, lastpoint + 2);
    this->triangles.push_back(newtri);
    return true;
}

bool ChConvexDecompositionHACD::AddTriangleMesh(const geometry::ChTriangleMesh& tm) {
    for (int i = 0; i < tm.getNumTriangles(); i++) {
        if (!this->ChConvexDecomposition::AddTriangle(tm.getTriangle(i)))
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
    myHACD->SetPoints(&this->points[0]);
    myHACD->SetNPoints(points.size());
    myHACD->SetTriangles(&this->triangles[0]);
    myHACD->SetNTriangles(triangles.size());

    myHACD->Compute();

    return (int)myHACD->GetNClusters();
}

/// Get the number of computed hulls after the convex decomposition
unsigned int ChConvexDecompositionHACD::GetHullCount() {
    return (unsigned int)this->myHACD->GetNClusters();
}

bool ChConvexDecompositionHACD::GetConvexHullResult(unsigned int hullIndex,
                                                    std::vector<ChVector<double> >& convexhull) {
    if (hullIndex > myHACD->GetNClusters())
        return false;

    size_t nPoints = myHACD->GetNPointsCH(hullIndex);
    size_t nTriangles = myHACD->GetNTrianglesCH(hullIndex);

    float* vertices = new float[nPoints * 3];
    unsigned int* triangles = new unsigned int[nTriangles * 3];

    HACD::Vec3<HACD::Real>* pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
    HACD::Vec3<long>* trianglesCH = new HACD::Vec3<long>[nTriangles];
    myHACD->GetCH(hullIndex, pointsCH, trianglesCH);

    // convert to chrono data...
    convexhull.clear();
    for (unsigned int i = 0; i < nPoints; i++) {
        ChVector<double> point(pointsCH[i].X(), pointsCH[i].Y(), pointsCH[i].Z());
        convexhull.push_back(point);
    }

    delete[] pointsCH;
    delete[] trianglesCH;

    return true;
}

/// Get the n-th computed convex hull, by filling a ChTriangleMesh object
/// that is passed as a parameter.
bool ChConvexDecompositionHACD::GetConvexHullResult(unsigned int hullIndex, geometry::ChTriangleMesh& convextrimesh) {
    if (hullIndex > myHACD->GetNClusters())
        return false;

    size_t nPoints = myHACD->GetNPointsCH(hullIndex);
    size_t nTriangles = myHACD->GetNTrianglesCH(hullIndex);

    float* vertices = new float[nPoints * 3];
    unsigned int* triangles = new unsigned int[nTriangles * 3];

    HACD::Vec3<HACD::Real>* pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
    HACD::Vec3<long>* trianglesCH = new HACD::Vec3<long>[nTriangles];
    myHACD->GetCH(hullIndex, pointsCH, trianglesCH);

    for (unsigned int i = 0; i < nTriangles; i++) {
        unsigned int i1 = trianglesCH[i].X();
        unsigned int i2 = trianglesCH[i].Y();
        unsigned int i3 = trianglesCH[i].Z();
        convextrimesh.addTriangle(ChVector<>(pointsCH[i1].X(), pointsCH[i1].Y(), pointsCH[i1].Z()),
                                  ChVector<>(pointsCH[i2].X(), pointsCH[i2].Y(), pointsCH[i2].Z()),
                                  ChVector<>(pointsCH[i3].X(), pointsCH[i3].Y(), pointsCH[i3].Z()));
    }

    delete[] pointsCH;
    delete[] trianglesCH;

    return true;
}

//
// SERIALIZATION
//

void ChConvexDecompositionHACD::WriteConvexHullsAsWavefrontObj(ChStreamOutAscii& mstream) {
    mstream << "# Convex hulls obtained with Chrono::Engine \n# convex decomposition \n\n";
    NxU32 vcount_base = 1;
    NxU32 vcount_total = 0;
    NxU32 tcount_total = 0;
    char buffer[200];
    for (unsigned int hullIndex = 0; hullIndex < this->GetHullCount(); hullIndex++) {
        mstream << "g hull_" << hullIndex << "\n";

        size_t nPoints = myHACD->GetNPointsCH(hullIndex);
        size_t nTriangles = myHACD->GetNTrianglesCH(hullIndex);

        float* vertices = new float[nPoints * 3];
        unsigned int* triangles = new unsigned int[nTriangles * 3];

        HACD::Vec3<HACD::Real>* pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
        HACD::Vec3<long>* trianglesCH = new HACD::Vec3<long>[nTriangles];
        myHACD->GetCH(hullIndex, pointsCH, trianglesCH);

        vcount_total += (NxU32)nPoints;
        tcount_total += (NxU32)nTriangles;
        for (unsigned int i = 0; i < nPoints; i++) {
            sprintf(buffer, "v %0.9f %0.9f %0.9f\r\n", pointsCH[i].X(), pointsCH[i].Y(), pointsCH[i].Z());
            mstream << buffer;
        }
        for (unsigned int i = 0; i < nTriangles; i++) {
            unsigned int i1 = trianglesCH[i].X();
            unsigned int i2 = trianglesCH[i].Y();
            unsigned int i3 = trianglesCH[i].Z();
            sprintf(buffer, "f %d %d %d\r\n", i1 + vcount_base, i2 + vcount_base, i3 + vcount_base);
            mstream << buffer;
        }
        vcount_base += (NxU32)nPoints;

        delete[] pointsCH;
        delete[] trianglesCH;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

//
//  ChConvexDecompositionJR
//

// CONVEX_DECOMPOSITION::iConvexDecomposition* ChConvexDecompositionJR::GetDecompositionObject() {return
// this->mydecomposition;}

ChConvexDecompositionJR::ChConvexDecompositionJR() {
    mydecomposition = CONVEX_DECOMPOSITION::createConvexDecomposition();
    skinWidth = 0;
    decompositionDepth = 8;
    maxHullVertices = 64;
    concavityThresholdPercent = 0.1f;
    mergeThresholdPercent = 30.0f;
    volumeSplitThresholdPercent = 0.1f;
    useInitialIslandGeneration = true;
    useIslandGeneration = false;
}

ChConvexDecompositionJR::~ChConvexDecompositionJR() {
    CONVEX_DECOMPOSITION::releaseConvexDecomposition(mydecomposition);
}

void ChConvexDecompositionJR::Reset(void) {
    this->mydecomposition->reset();
}

bool ChConvexDecompositionJR::AddTriangle(const ChVector<>& v1, const ChVector<>& v2, const ChVector<>& v3) {
    NxF32 p1[3];
    p1[0] = (float)v1.x();
    p1[1] = (float)v1.y();
    p1[2] = (float)v1.z();
    NxF32 p2[3];
    p2[0] = (float)v2.x();
    p2[1] = (float)v2.y();
    p2[2] = (float)v2.z();
    NxF32 p3[3];
    p3[0] = (float)v3.x();
    p3[1] = (float)v3.y();
    p3[2] = (float)v3.z();
    return this->mydecomposition->addTriangle(p1, p2, p3);
}

bool ChConvexDecompositionJR::AddTriangleMesh(const geometry::ChTriangleMesh& tm) {
    for (int i = 0; i < tm.getNumTriangles(); i++) {
        if (!this->ChConvexDecomposition::AddTriangle(tm.getTriangle(i)))
            return false;
    }
    return true;
}

void ChConvexDecompositionJR::SetParameters(
    float mskinWidth,                    ///< Skin width on the convex hulls generated
    unsigned int mdecompositionDepth,    ///< Recursion depth for convex decomposition.
    unsigned int mmaxHullVertices,       ///< Maximum number of vertices in output convex hulls.
    float mconcavityThresholdPercent,    ///< The percentage of concavity allowed without causing a split to occur.
    float mmergeThresholdPercent,        ///< The percentage of volume difference allowed to merge two convex hulls.
    float mvolumeSplitThresholdPercent,  ///< The percentage of the total volume of the object above which splits will
    /// still occur.
    bool museInitialIslandGeneration,  ///< Whether or not to perform initial island generation on the input mesh.
    bool museIslandGeneration  ///< Whether or not to perform island generation at each split.  Currently disabled.
    ) {
    skinWidth = mskinWidth;
    decompositionDepth = mdecompositionDepth;
    maxHullVertices = mmaxHullVertices;
    concavityThresholdPercent = mconcavityThresholdPercent;
    mergeThresholdPercent = mmergeThresholdPercent;
    volumeSplitThresholdPercent = mvolumeSplitThresholdPercent;
    useInitialIslandGeneration = museInitialIslandGeneration;
    useIslandGeneration = museIslandGeneration;
}

int ChConvexDecompositionJR::ComputeConvexDecomposition() {
    return this->mydecomposition->computeConvexDecomposition(
        skinWidth, decompositionDepth, maxHullVertices, concavityThresholdPercent, mergeThresholdPercent,
        volumeSplitThresholdPercent, useInitialIslandGeneration, useIslandGeneration, false);
}

/// Get the number of computed hulls after the convex decomposition
unsigned int ChConvexDecompositionJR::GetHullCount() {
    return this->mydecomposition->getHullCount();
}

/// Get the n-th computed convex hull, by filling a ChTriangleMesh object
/// that is passed as a parameter.
bool ChConvexDecompositionJR::GetConvexHullResult(unsigned int hullIndex, geometry::ChTriangleMesh& convextrimesh) {
    CONVEX_DECOMPOSITION::ConvexHullResult result;
    if (!this->mydecomposition->getConvexHullResult(hullIndex, result))
        return false;

    for (unsigned int i = 0; i < result.mTcount; i++) {
        unsigned int i1 = 3 * result.mIndices[i * 3 + 0];
        unsigned int i2 = 3 * result.mIndices[i * 3 + 1];
        unsigned int i3 = 3 * result.mIndices[i * 3 + 2];
        convextrimesh.addTriangle(
            ChVector<>(result.mVertices[i1 + 0], result.mVertices[i1 + 1], result.mVertices[i1 + 2]),
            ChVector<>(result.mVertices[i2 + 0], result.mVertices[i2 + 1], result.mVertices[i2 + 2]),
            ChVector<>(result.mVertices[i3 + 0], result.mVertices[i3 + 1], result.mVertices[i3 + 2]));
    }
    return true;
}

bool ChConvexDecompositionJR::GetConvexHullResult(unsigned int hullIndex, std::vector<ChVector<double> >& convexhull) {
    CONVEX_DECOMPOSITION::ConvexHullResult result;
    if (!this->mydecomposition->getConvexHullResult(hullIndex, result))
        return false;

    // convert to chrono data...
    convexhull.clear();
    for (unsigned int i = 0; i < result.mVcount; i++) {
        ChVector<double> point(result.mVertices[i * 3 + 0], result.mVertices[i * 3 + 1], result.mVertices[i * 3 + 2]);
        convexhull.push_back(point);
    }

    return true;
}

//
// SERIALIZATION
//

/// Save the computed convex hulls as a Wavefront file using the
/// '.obj' fileformat, with each hull as a separate group.
/// May throw exceptions if file locked etc.
void ChConvexDecompositionJR::WriteConvexHullsAsWavefrontObj(ChStreamOutAscii& mstream) {
    mstream << "# Convex hulls obtained with Chrono::Engine \n# convex decomposition \n\n";
    NxU32 vcount_base = 1;
    NxU32 vcount_total = 0;
    NxU32 tcount_total = 0;
    char buffer[200];
    for (NxU32 i = 0; i < this->GetHullCount(); i++) {
        mstream << "g hull_" << i << "\n";
        CONVEX_DECOMPOSITION::ConvexHullResult result;
        this->mydecomposition->getConvexHullResult(i, result);
        vcount_total += result.mVcount;
        tcount_total += result.mTcount;
        for (NxU32 i = 0; i < result.mVcount; i++) {
            const NxF32* pos = &result.mVertices[i * 3];
            sprintf(buffer, "v %0.9f %0.9f %0.9f\r\n", pos[0], pos[1], pos[2]);
            mstream << buffer;
        }
        for (NxU32 i = 0; i < result.mTcount; i++) {
            NxU32 i1 = result.mIndices[i * 3 + 0];
            NxU32 i2 = result.mIndices[i * 3 + 1];
            NxU32 i3 = result.mIndices[i * 3 + 2];
            sprintf(buffer, "f %d %d %d\r\n", i1 + vcount_base, i2 + vcount_base, i3 + vcount_base);
            mstream << buffer;
        }
        vcount_base += result.mVcount;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

//
//  ChConvexDecompositionHACDv2
//

/// Basic constructor
ChConvexDecompositionHACDv2::ChConvexDecompositionHACDv2() {
    this->descriptor.init();

    gHACD = HACD::createHACD_API();

    this->fuse_tol = 1e-9;
}

/// Destructor
ChConvexDecompositionHACDv2::~ChConvexDecompositionHACDv2() {
    gHACD->release();  // will delete itself
}

void ChConvexDecompositionHACDv2::Reset(void) {
    this->descriptor.init();
    gHACD->releaseHACD();

    this->points.clear();
    this->triangles.clear();
}

bool ChConvexDecompositionHACDv2::AddTriangle(const ChVector<>& v1, const ChVector<>& v2, const ChVector<>& v3) {
    int lastpoint = (int)this->points.size();
    ChVector<double> vertex1(v1.x(), v1.y(), v1.z());
    ChVector<double> vertex2(v2.x(), v2.y(), v2.z());
    ChVector<double> vertex3(v3.x(), v3.y(), v3.z());
    this->points.push_back(vertex1);
    this->points.push_back(vertex2);
    this->points.push_back(vertex3);
    ChVector<int> newtri(lastpoint, lastpoint + 1, lastpoint + 2);
    this->triangles.push_back(newtri);
    return true;
}

bool ChConvexDecompositionHACDv2::AddTriangleMesh(const geometry::ChTriangleMesh& tm) {
    for (int i = 0; i < tm.getNumTriangles(); i++) {
        if (!this->ChConvexDecomposition::AddTriangle(tm.getTriangle(i)))
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
    // this->descriptor.init();
    this->descriptor.mMaxHullCount = mmMaxHullCount;
    this->descriptor.mMaxMergeHullCount = mmMaxMergeHullCount;
    this->descriptor.mMaxHullVertices = mmMaxHullVertices;
    this->descriptor.mConcavity = mmConcavity;
    this->descriptor.mSmallClusterThreshold = mmSmallClusterThreshold;
    this->fuse_tol = mmFuseTol;
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

    // Preprocess: fuse repeated vertices...

    std::vector<ChVector<double> > points_FUSED;
    std::vector<ChVector<int> > triangles_FUSED;
    FuseMesh(this->points, this->triangles, points_FUSED, triangles_FUSED, this->fuse_tol);

    // Convert to HACD format

    this->descriptor.mTriangleCount = (hacd::HaU32)triangles_FUSED.size();
    this->descriptor.mVertexCount = (hacd::HaU32)points_FUSED.size();
    this->descriptor.mIndices = new hacd::HaU32[3 * this->descriptor.mTriangleCount];
    this->descriptor.mVertices = new hacd::HaF32[3 * this->descriptor.mVertexCount];
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

    // Perform the decomposition!

    hacd::HaU32 hullCount = gHACD->performHACD(this->descriptor);

    delete[] this->descriptor.mIndices;
    delete[] this->descriptor.mVertices;
    this->descriptor.mTriangleCount = 0;
    this->descriptor.mVertexCount = 0;

    return hullCount;
}

/// Get the number of computed hulls after the convex decomposition
unsigned int ChConvexDecompositionHACDv2::GetHullCount() {
    return this->gHACD->getHullCount();
}

bool ChConvexDecompositionHACDv2::GetConvexHullResult(unsigned int hullIndex,
                                                      std::vector<ChVector<double> >& convexhull) {
    if (hullIndex > this->gHACD->getHullCount())
        return false;

    const HACD::HACD_API::Hull* hull = gHACD->getHull(hullIndex);
    if (hull) {
        for (hacd::HaU32 i = 0; i < hull->mVertexCount; i++) {
            const hacd::HaF32* p = &hull->mVertices[i * 3];
            ChVector<double> point(p[0], p[1], p[2]);
            convexhull.push_back(point);
        }
        return true;
    }
    return false;
}

/// Get the n-th computed convex hull, by filling a ChTriangleMesh object
/// that is passed as a parameter.
bool ChConvexDecompositionHACDv2::GetConvexHullResult(unsigned int hullIndex, geometry::ChTriangleMesh& convextrimesh) {
    if (hullIndex > this->gHACD->getHullCount())
        return false;

    const HACD::HACD_API::Hull* hull = gHACD->getHull(hullIndex);
    if (hull) {
        for (unsigned int i = 0; i < hull->mTriangleCount; i++) {
            unsigned int i1 = 3 * hull->mIndices[i * 3 + 0];
            unsigned int i2 = 3 * hull->mIndices[i * 3 + 1];
            unsigned int i3 = 3 * hull->mIndices[i * 3 + 2];
            convextrimesh.addTriangle(
                ChVector<>(hull->mVertices[i1 + 0], hull->mVertices[i1 + 1], hull->mVertices[i1 + 2]),
                ChVector<>(hull->mVertices[i2 + 0], hull->mVertices[i2 + 1], hull->mVertices[i2 + 2]),
                ChVector<>(hull->mVertices[i3 + 0], hull->mVertices[i3 + 1], hull->mVertices[i3 + 2]));
        }
    }
    return true;
}

//
// SERIALIZATION
//

void ChConvexDecompositionHACDv2::WriteConvexHullsAsWavefrontObj(ChStreamOutAscii& mstream) {
    mstream << "# Convex hulls obtained with Chrono::Engine \n# convex decomposition \n\n";

    char buffer[200];

    hacd::HaU32* baseVertex = new hacd::HaU32[GetHullCount()];
    hacd::HaU32 vertexCount = 0;
    for (hacd::HaU32 i = 0; i < GetHullCount(); i++) {
        const HACD::HACD_API::Hull* hull = gHACD->getHull(i);
        if (hull) {
            baseVertex[i] = vertexCount;
            for (hacd::HaU32 i = 0; i < hull->mVertexCount; i++) {
                const hacd::HaF32* p = &hull->mVertices[i * 3];
                sprintf(buffer, "v %0.9f %0.9f %0.9f\r\n", p[0], p[1], p[2]);
                mstream << buffer;
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
                sprintf(buffer, "f %d %d %d\r\n", i1, i2, i3);
                mstream << buffer;
            }
        }
    }
    delete[] baseVertex;
}

}  // end namespace collision
}  // end namespace chrono
