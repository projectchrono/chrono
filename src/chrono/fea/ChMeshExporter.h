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
// Authors: Milad Rakhsha
// =============================================================================
#ifndef CHMESHEXPORTER_H_
#define CHMESHEXPORTER_H_

#include "chrono/fea/ChElementHexaANCF_3813.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyz.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_utils
/// @{

/// Collection of mesh file writer utilities.

class ChApi ChMeshExporter {
  public:
    /// This function is used to write the connectivity and information of a ChMesh
    /// Currently it only supports ChElementCableANCF, ChElementShellANCF_3423, ChElementHexaANCF_3813
    /// This connectivity is usually constant throughout the simulation. Hence, it may be
    /// called once at the beginning of the simulation (or as many times as it is needed),
    /// and should be used in the later stage when the nodal/elemental informations are updated.
    static void writeMesh(std::shared_ptr<ChMesh> my_mesh,  ///< destination mesh
                          std::string SaveAs                ///< name of the mesh file
    );

    /// The task of this function is to write the information of a ChMesh into a file with vtk format.
    /// The vtk files may be opened with external visualization software such as Paraview, Visit, etc.
    /// This function requires the mesh connectivity file, which may be written by ChMeshExporter::writeMesh function
    /// Note that this function writes the current state of the mesh when is called. It writes some nodal
    /// informations (see the implementation) and elemental informations at the "element center". The elemental
    /// information may be converted to nodal information later on in the post-processing stage with a visualization
    /// software like Paraview.
    static void writeFrame(std::shared_ptr<ChMesh> my_mesh,  ///< destination mesh
                           char SaveAsBuffer[256],           ///< name of the vtk file to export data to
                           std::string MeshFileBuffer  ///< name of the mesh file written by ChMeshExporter::writeMesh
    );
};

/// @} fea_utils

}  // namespace fea
}  // namespace chrono

#endif
