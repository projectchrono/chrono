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
#ifndef CH_MESH_EXPORTER_H
#define CH_MESH_EXPORTER_H

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
    /// Write FEA mesh connectivity to specified output file.
    /// This connectivity is usually constant throughout the simulation. Hence, it may be called once at the beginning
    /// of the simulation (or as many times as it is needed), and should be used in the later stage when the
    /// nodal/elemental informations are updated.
    static void WriteMesh(std::shared_ptr<ChMesh> mesh,     ///< destination mesh
                          const std::string& mesh_filename  ///< output file name
    );

    /// Write current FEA mesh information in VTK format.
    /// The vtk files may be opened with external visualization software such as Paraview, Visit, etc.
    /// This function requires the mesh connectivity file, which may be written by ChMeshExporter::writeMesh function.
    /// Note that this function writes the current state of the mesh when is called. It writes some nodal
    /// and elemental information at the "element center". The elemental information may be converted to nodal
    /// information later on in the post-processing stage with a visualization software like Paraview.
    static void WriteFrame(std::shared_ptr<ChMesh> mesh,      ///< destination mesh
                           const std::string& mesh_filename,  ///< name of the mesh file with connectivity information
                           const std::string& vtk_filename    ///< output file name
    );
};

/// @} fea_utils

}  // namespace fea
}  // namespace chrono

#endif
