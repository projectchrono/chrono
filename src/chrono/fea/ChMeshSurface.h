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

#ifndef CHMESHSURFACE_H
#define CHMESHSURFACE_H

#include "chrono/physics/ChLoadable.h"
#include "chrono/fea/ChElementBase.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{

// Forward references (for parent hierarchy pointer)
class ChMesh;

/// Class which defines a surface for a mesh FEA elements.
/// The surface is a collection of pointers to  ChLoadableUV objects, which can be shells in the mesh or proxies
/// to faces of solid elements (such as ChTetrahedronFace or ChHexahedronFace).
class ChApi ChMeshSurface {
  public:
    ChMeshSurface(ChMesh* parentmesh = nullptr) : mmesh(parentmesh) {}
    virtual ~ChMeshSurface() {}

    /// Get owner mesh.
    ChMesh* GetMesh() { return mmesh; }

    /// Set owner mesh.
    void SetMesh(ChMesh* mm) { mmesh = mm; }

    /// Direct access to the list of faces.
    std::vector<std::shared_ptr<ChLoadableUV> >& GetFacesList() { return faces; }

    /// Add a single mesh face.
    /// Note that this function does not check for double insertion of the same face.
    virtual void AddFace(std::shared_ptr<ChLoadableUV> mface) { faces.push_back(mface); }

    /// Add multiple faces of FEM elements given a set of nodes at vertexes.
    /// Scan all the finite elements already added in the parent ChMesh, and check if any has a face whose vertexes are
    /// all in the given node set; if so, add it to this mesh surface, with these rules:
    /// - surface elements inherited from ChLoadableUV: the element is added
    /// - face of ChElementTetrahedron : a ChTetrahedronFace proxy is created and added
    /// - face of ChElementHexahedron : a ChHexahedronFace proxy is created and added
    virtual void AddFacesFromNodeSet(std::vector<std::shared_ptr<ChNodeFEAbase> >& node_set);

    /// Find faces on the outer boundary of a solid mesh.
    /// Scan all the finite elements already added in the parent ChMesh and add the faces that are not shared.
    virtual void AddFacesFromBoundary();

  private:
    std::vector<std::shared_ptr<ChLoadableUV> > faces;  ///< mesh faces
    ChMesh* mmesh;                                      ///< parent mesh
};

/// @} chrono_fea

}  // end namespace fea
}  // end namespace chrono

#endif
