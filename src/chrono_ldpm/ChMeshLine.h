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

#ifndef CHMESHLINES_H
#define CHMESHLINES_H

#include "chrono/physics/ChLoadable.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChElementBase.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChBeamSectionEuler.h"
#include "chrono/fea/ChBuilderBeam.h"

using namespace chrono;
using namespace fea;

namespace chrono {
namespace ldpm {

/// @addtogroup chrono_fea
/// @{

// Forward references (for parent hierarchy pointer)
//class chrono::fea::ChMesh;

/// Class which defines a line for a mesh FEA elements.
/// The line is a collection of pointers to  ChLoadableU objects, which can be beams in the mesh.
class ChApi ChMeshLine {
  public:
    ChMeshLine(ChMesh* parentmesh = nullptr) : mmesh(parentmesh) {}
    virtual ~ChMeshLine() {}

    /// Get owner mesh.
    ChMesh* GetMesh() { return mmesh; }

    /// Set owner mesh.
    void SetMesh(ChMesh* mm) { mmesh = mm; }

    /// Direct access to the list of beams.
    std::vector<std::shared_ptr<ChLoadableU> >& GetBeamsList() { return beams; }

    /// Add a single mesh beam.
    /// Note that this function does not check for double insertion of the same beam.
    virtual void AddBeam(std::shared_ptr<ChLoadableU> mbeam) { beams.push_back(mbeam); }

    /// Add multiple beams of FEM elements given a set of nodes at vertexes.
    /// Scan all the finite elements already added in the parent ChMesh, and check if any has a line whose vertexes are
    /// all in the given node set; if so, add it to this mesh line.
    //virtual void AddBeamsFromNodeSet(std::vector<std::shared_ptr<ChNodeFEAbase> >& node_set);
    virtual void AddBeamsFromNodeSet(std::vector<std::shared_ptr<fea::ChNodeFEAxyzrot> >& node_set);
    
    virtual void AddBeamsFromBoundary();
    
    static void SortNodesByCoordinates(std::vector<std::shared_ptr<fea::ChNodeFEAbase>>& node_set);

  private:
    std::vector<std::shared_ptr<ChLoadableU> > beams;  ///< mesh beams
    chrono::fea::ChMesh* mmesh;                                            ///< parent mesh
};

/// @} chrono_fea

}  // end namespace ldpm
}  // end namespace chrono

#endif

