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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHMESHSURFACE_H
#define CHMESHSURFACE_H

#include "chrono/physics/ChLoadable.h"
#include "chrono_fea/ChElementBase.h"

namespace chrono {
namespace fea {

// Forward references (for parent hierarchy pointer)
class ChMesh;

/// Class which defines a surface for a mesh FEA elements.
/// The contact surface is a collection of pointers to  ChLoadableUV objects, those can
/// be shells in the mesh, or proxies to faces of solid elements such as ChFaceTetra_4.
class ChApiFea ChMeshSurface {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChMeshSurface)

  public:
    ChMeshSurface(ChMesh* parentmesh = 0) { mmesh = parentmesh; }

    virtual ~ChMeshSurface() {}

    //
    // FUNCTIONS
    //

    /// Get owner mesh
    ChMesh* GetMesh() { return mmesh; }

    /// Set owner mesh
    void SetMesh(ChMesh* mm) { mmesh = mm; }

    /// Direct access to the list of faces
    std::vector<std::shared_ptr<ChLoadableUV> >& GetFacesList() { return faces; }

    /// Add a single mesh face.
    /// Beware, it does not check for double insertion of the same face.
    virtual void AddFace(std::shared_ptr<ChLoadableUV> mface) { faces.push_back(mface); }

    /// Add multiple faces of FEM elements given a set of nodes at vertexes.
    /// It scans all the finite elements already added in the parent ChMesh, and
    /// see if someone has a face whose vertexes are all in the given node set;
    /// if so, adds it to this mesh surface, with these rules:
    /// - surface elements inherited from ChLoadableUV such as ChElementShellANCF: the element is added
    /// - face of ChElementTetra_4 : a ChFaceTetra_4 proxy is created and added
    /// - Support for other elements of solid type will follow in future.
    virtual void AddFacesFromNodeSet(std::vector<std::shared_ptr<ChNodeFEAbase> >& node_set);

    /// Given a solid mesh (ex a mesh of tetrahedrons) it finds the faces on the outer boundary.
    /// That is, it scans all the finite elements already added in the parent ChMesh and adds the faces
    /// that are not shared (ie. the faces on the boundary 'skin').
    /// Supported solids that generate boundary skin:
    /// - tetrahedrons
    /// - more will follow in future
    virtual void AddFacesFromBoundary();

  private:
    std::vector<std::shared_ptr<ChLoadableUV> > faces;  // the faces

    std::shared_ptr<ChMaterialSurface> matsurface;  // material for contacts

    ChMesh* mmesh;
};

}  // end namespace fea
}  // end namespace chrono

#endif
