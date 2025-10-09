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
// Authors: Andrea Favali, Alessandro Tasora
// =============================================================================
// Utilities for loading meshes from file
// =============================================================================

#ifndef CHMESH_FILE_LOADER_BEAM_H
#define CHMESH_FILE_LOADER_BEAM_H

#include <map>

#include "chrono_flow/ChElementSpringPPP.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChElementShellBST.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/core/ChMatrix.h"
#include "chrono_flow/ChFlowApi.h"

using namespace chrono::fea;

namespace chrono {
namespace flow {

/// @addtogroup fea_utils
/// @{

/// Collection of mesh file loader utilities.
class ChFlowApi ChMeshFileLoaderBeam {
  public:
  /// Load mesh information imported from various software, for use as generic FEA mechanical or thermal tets,
  /// or specific flow elements (in the case of CBLMultiMat, with different materials for longitudinal and lateral elements).

    // Similar to FromAbaqusFile but for FreeCAD exported .inp files for single material meshes, imports fea, or flow pipes
    static void FromFreeCADFile( 
        std::shared_ptr<ChMesh> mesh,                      /// destination mesh
        const char* filename,                              /// input file name
        std::shared_ptr<ChContinuumMaterial> my_material,  /// material for the created tetahedrons
        std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase> > >&
            node_sets,                                     /// vect of vectors of 'marked'nodes
        ChVector3d pos_transform = VNULL,                  /// optional displacement of imported mesh
        ChMatrix33<> rot_transform = ChMatrix33<>(1),      /// optional rotation/scaling of imported mesh
        bool discard_unused_nodes =
            true  ///< if true, Abaqus nodes that are not used in elements or sets are not imported in C::E
    );
    // Imports pipes for use in CBL flow elements, with different materials for longitudinal and lateral elements based on FreeCAD export
    static void FromFreeCADFileCBLMultiMat(
        std::shared_ptr<ChMesh> mesh,                       /// destination mesh
        const char* filename,                               /// input file name
        std::shared_ptr<ChContinuumMaterial> matLong,       /// material for longitudinal elements
        std::shared_ptr<ChContinuumMaterial> matTrans,        /// material for lateral elements
        std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase> > >&
            node_sets,                                      /// vect of vectors of 'marked'nodes
        ChVector3d pos_transform = VNULL,                   /// optional displacement of imported mesh
        ChMatrix33<> rot_transform = ChMatrix33<>(1),       /// optional rotation/scaling of imported mesh
        bool discard_unused_nodes =
            true  ///< if true, Abaqus nodes that are not used in elements or sets are not imported in C::E
    );

    /// Load tetrahedrons, if any, saved in a .inp file for Abaqus.
    static void FromAbaqusFile(
        std::shared_ptr<ChMesh> mesh,                      ///< destination mesh
        const char* filename,                              ///< input file name
        std::shared_ptr<ChContinuumMaterial> my_material,  ///< material for the created tetahedrons
        std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase> > >&
            node_sets,                                 ///< vect of vectors of 'marked'nodes
        ChVector3d pos_transform = VNULL,              ///< optional displacement of imported mesh
        ChMatrix33<> rot_transform = ChMatrix33<>(1),  ///< optional rotation/scaling of imported mesh
        bool discard_unused_nodes =
            true  ///< if true, Abaqus nodes that are not used in elements or sets are not imported in C::E
    );

};

/// @} fea_utils

}  // end namespace flow
}  // end namespace chrono

#endif
