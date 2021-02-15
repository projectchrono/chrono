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
// Authors: Alesandro Tasora, Radu Serban
// =============================================================================

#ifndef CHTRIANGLEMESHSHAPE_H
#define CHTRIANGLEMESHSHAPE_H

#include <vector>

#include "chrono/assets/ChVisualization.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace chrono {

/// Class for referencing a triangle mesh shape that can be visualized in some way.
/// As a child class of ChAsset, it can be 'attached' to physics items.
/// It also defines flags such as 'draw as wireframe', 'do backface culling' etc.
/// but remember that depending on the type of visualization system
/// (POVray, Irrlich,etc.) these flags might not be supported.
class ChApi ChTriangleMeshShape : public ChVisualization {
  public:
    ChTriangleMeshShape();
    ~ChTriangleMeshShape() {}

    std::shared_ptr<geometry::ChTriangleMeshConnected> GetMesh() { return trimesh; }
    void SetMesh(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh) { trimesh = mesh; }

    bool IsWireframe() const { return wireframe; }
    void SetWireframe(bool mw) { wireframe = mw; }

    bool IsBackfaceCull() const { return backface_cull; }
    void SetBackfaceCull(bool mbc) { backface_cull = mbc; }

    const std::string& GetName() const { return name; }
    void SetName(const std::string& mname) { name = mname; }

    const ChVector<>& GetScale() const { return scale; }
    void SetScale(const ChVector<>& mscale) { scale = mscale; }

    void SetFixedConnectivity() { fixed_connectivity = true; }
    bool FixedConnectivity() const { return fixed_connectivity; }
    void SetModifiedVertices(std::vector<int> vertices) { modified_vertices = vertices; }
    const std::vector<int>& GetModifiedVertices() const { return modified_vertices; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  protected:
    std::shared_ptr<geometry::ChTriangleMeshConnected> trimesh;

    bool wireframe;
    bool backface_cull;

    std::string name;
    ChVector<> scale;

    bool fixed_connectivity;
    std::vector<int> modified_vertices;
};

CH_CLASS_VERSION(ChTriangleMeshShape, 0)

}  // end namespace chrono

#endif
