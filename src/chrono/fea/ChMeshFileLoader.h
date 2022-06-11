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

#ifndef CHMESH_FILE_LOADER_H
#define CHMESH_FILE_LOADER_H

#include <map>

#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChElementShellBST.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/core/ChMatrix.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_utils
/// @{

/// Collection of mesh file loader utilities.
class ChApi ChMeshFileLoader {
  public:
    /// Load tetrahedrons from .node and .ele files as saved by TetGen.
    /// The file format for .node (with point# starting from 1) is:
    ///   [# of points] [dimension (only 3)] [# of attributes (only 0)] [markers (only 0)]
    ///   [node #] [x] [y] [z]
    ///   [node #] [x] [y] [z]   etc.
    /// The file format for .ele (with tet# starting from 1) is:
    ///   [# of tetrahedrons] [dimension (only 4 supported)] [# of attributes (only 0)]
    ///   [tet #] [node #] [node #] [node #] [node #]
    ///   [tet #] [node #] [node #] [node #] [node #]   etc.
    /// If you pass a material inherited by ChContinuumElastic, nodes with 3D motion are used, and corotational
    /// elements.
    /// If you pass a material inherited by ChContinuumPoisson3D, nodes with scalar field are used (ex. thermal,
    /// electrostatics, etc)
    static void FromTetGenFile(
        std::shared_ptr<ChMesh> mesh,                      ///< destination mesh
        const char* filename_node,                         ///< name of the .node file
        const char* filename_ele,                          ///< name of the .ele  file
        std::shared_ptr<ChContinuumMaterial> my_material,  ///< material for the created tetahedrons
        ChVector<> pos_transform = VNULL,                  ///< optional displacement of imported mesh
        ChMatrix33<> rot_transform = ChMatrix33<>(1)       ///< optional rotation/scaling of imported mesh
    );

    /// Load tetrahedrons, if any, saved in a .inp file for Abaqus.
    static void FromAbaqusFile(
        std::shared_ptr<ChMesh> mesh,                      ///< destination mesh
        const char* filename,                              ///< input file name
        std::shared_ptr<ChContinuumMaterial> my_material,  ///< material for the created tetahedrons
        std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase> > >&
            node_sets,                                 ///< vect of vectors of 'marked'nodes
        ChVector<> pos_transform = VNULL,              ///< optional displacement of imported mesh
        ChMatrix33<> rot_transform = ChMatrix33<>(1),  ///< optional rotation/scaling of imported mesh
        bool discard_unused_nodes =
            true  ///< if true, Abaqus nodes that are not used in elements or sets are not imported in C::E
    );

    static void ANCFShellFromGMFFile(
        std::shared_ptr<ChMesh> mesh,                      ///< destination mesh
        const char* filename,                              ///< complete filename
        std::shared_ptr<ChMaterialShellANCF> my_material,  ///< material to be given to the shell
        std::vector<double>& node_ave_area,                ///< output the average area of the nodes
        std::vector<int>& BC_nodes,                        ///< material to be given to the shell
        ChVector<> pos_transform = VNULL,                  ///< optional displacement of imported mesh
        ChMatrix33<> rot_transform = ChMatrix33<>(1),      ///< optional rotation/scaling of imported mesh
        double scaleFactor = 1,                            ///< import scale factor
        bool printNodes = false,                           ///< display the imported nodes
        bool printElements = false                         ///< display the imported elements
    );

    /// Load a triangle mesh in Wavefront OBJ file format, and convert it into a mesh of shell elements of ChElementShellBST type.
    static void BSTShellFromObjFile(
        std::shared_ptr<ChMesh> mesh,                           ///< destination mesh
        const char* filename,                                   ///< .obj mesh complete filename
        std::shared_ptr<ChMaterialShellKirchhoff> my_material,  ///< material to be given to the shell elements
        double my_thickness,                                    ///< thickness to be given to shell elements
        ChVector<> pos_transform = VNULL,                       ///< optional displacement of imported mesh
        ChMatrix33<> rot_transform = ChMatrix33<>(1)            ///< optional rotation/scaling of imported mesh
    );
};

/// @} fea_utils

}  // end namespace fea
}  // end namespace chrono

#endif
