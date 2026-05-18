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

#ifndef CHFEMODELSURFACE_H
#define CHFEMODELSURFACE_H

#include "chrono/physics/ChLoadable.h"
#include "chrono/fea/multiphysics/ChFieldElement.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{

// Forward references (for parent hierarchy pointer)
class ChFEModel;

/// Class which defines a surface for a model of FEA elements.
/// The surface is a collection of pointers to  ChFieldElementSurface objects, which can be shells in the model or proxies
/// to faces of solid elements (such as ChFieldElementTetrahedron4Face or ChFieldElementHexahedron8Face).
/// Those faces can be wrapped into ChFieldElementLoadableSurface if one aims at loading them via pressure or other loads. 

class ChApi ChSurfaceOfModel {
  public:
    ChSurfaceOfModel(ChFEModel* parentmodel = nullptr) : mmodel(parentmodel) {}
    virtual ~ChSurfaceOfModel() {}

    /// Get owner model.
    ChFEModel* Getmodel() { return mmodel; }

    /// Set owner model.
    void Setmodel(ChFEModel* mm) { mmodel = mm; }

    /// Direct access to the list of faces.
    std::vector<std::shared_ptr<ChFieldElementSurface> >& GetFaces() { return faces; }

    /// Add a single model face.
    /// Note that this function does not check for double insertion of the same face.
    virtual void AddFace(std::shared_ptr<ChFieldElementSurface> mface) { faces.push_back(mface); }

    /// Add multiple faces of FEM elements given a set of nodes at vertexes.
    /// Scan all the finite elements already added in the parent ChFEModel, and check if any has a face whose vertexes are
    /// all in the given node set; if so, add it to this model surface.
    virtual void AddFacesFromNodeSet(std::vector<std::shared_ptr<ChNodeFEAbase> >& node_set);

    /// Find faces on the outer boundary of a solid model.
    /// Scan all the finite elements already added in the parent ChFEModel and add the faces that are not shared.
    virtual void AddFacesFromBoundary();

  private:
    std::vector<std::shared_ptr<ChFieldElementSurface> > faces;      ///< model faces
    ChFEModel* mmodel;                                      ///< parent model
};

/// @} chrono_fea

}  // end namespace fea
}  // end namespace chrono

#endif
