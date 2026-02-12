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
// Authors: Radu Serban
// =============================================================================
//
// Utility functions for input/output file operations.
//
// WriteCheckpoint and ReadCheckpoint
//  these functions write and read, respectively, a checkpoint file.
//  Limitations:
//    - it is assumed that the visualization asset geometry exctly matches the
//      contact geometry.
//    - only a subset of contact shapes are currently supported
//
// WriteVisualizationAssets
//  this function writes a CSV file appropriate for processing with a POV-Ray
//  script.
//
// =============================================================================

#ifndef CH_UTILS_INOUT_H
#define CH_UTILS_INOUT_H

#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <functional>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChBezierCurve.h"
#include "chrono/assets/ChColor.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsCreators.h"

namespace chrono {
namespace utils {

/// @addtogroup chrono_io
/// @{

/// Create a CSV file with a checkpoint of bodies and collision shapes.
/// This is a specialized checkpointing function suitable for DEM simulations (e.g., with Chrono::Multicore). Note that
/// only body states are checkpointed and therefore this function should not be used for Chrono multibody systems.
ChApi bool WriteBodyShapesCheckpoint(ChSystem* system, const std::string& filename);

/// Read a CSV file with a checkpoint of bodies and collision shapes.
/// This function creates bodies in the given Chrono system using states and collision shapes read from the checkpoint
/// file with given name (assumed to have been produced with WriteBodyShapesCheckpoint).
/// This is a specialized checkpointing function suitable for DEM simulations (e.g., with Chrono::Multicore). Note that
/// only body states are checkpointed and therefore this function should not be used for Chrono multibody systems.
ChApi void ReadBodyShapesCheckpoint(ChSystem* system, const std::string& filename);

/// Write CSV output file with body and asset information for off-line visualization.
/// (1) The first line of the output file contains:
/// - the number of bodies (0 if no body information is included below)
/// - the number of visual assets
/// - the number of links
/// - the number of TSDA spring-dampers
/// (2) If the number of bodies is not zero, the next block of lines includes information about the rigid bodies. Each
/// line contains 9 values:
/// - the body identifier
/// - a flag (1 or 0) indicating whether the body is active or not
/// - the body position expressed in global frame
/// - the body orientation (as a quaternion, expressed in global frame)
/// (3) The next block of lines includes information about the visual assets. Each line contains:
/// - the identifier of the associated body
/// - a flag (1 or 0) indicating whether the body is active or not
/// - the asset position expresssed in global frame
/// - the asset orientation (as a quaternion, expressed in global frame)
/// - the visualization asset type (an enum value)
/// - geometry information depending on the type above
/// (4) The next block of lines includes information about the joints. Each line contains:
/// - the joint type (an enum value)
/// - joint position and orientation information depending on the type above
/// (5) The next block of lines includes information about TSDA elements. Each line contains:
/// - the type of visual asset (0 for segment or 1 for coil)
/// - start point position
/// - end point position
ChApi void WriteVisualizationAssets(ChSystem* system,               ///< containg system
                                    const std::string& filename,    ///< output file name
                                    bool body_info = true,          ///< include body state information
                                    const std::string& delim = ","  ///< CSV delimitator
);

/// Write CSV output file with body and asset information for off-line visualization.
/// This version uses a discriminator function to select which bodies and assets are included in the output.
/// The function selector receives as argument a body reference and must return 'true' if the body and its assets should
/// be included in the output file and 'false' otherwise.
ChApi void WriteVisualizationAssets(ChSystem* system,                             ///< containg system
                                    const std::string& filename,                  ///< output file name
                                    std::function<bool(const ChBody&)> selector,  ///< select bodies
                                    bool body_info = true,                        ///< include body state information
                                    const std::string& delim = ","                ///< CSV delimitator
);

/// Write the specified mesh as a macro in a PovRay include file. The output file will be "[out_dir]/[mesh_name].inc".
/// The mesh vertices will be transformed to the frame with specified offset and orientation.
ChApi void WriteMeshPovray(ChTriangleMeshConnected& trimesh,
                           const std::string& mesh_name,
                           const std::string& out_dir,
                           const ChColor& color = ChColor(0.4f, 0.4f, 0.4f),
                           const ChVector3d& pos = ChVector3d(0, 0, 0),
                           const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                           bool smoothed = false);

/// Write the triangular mesh from the specified OBJ file as a macro in a PovRay include file. The output file will be
/// "[out_dir]/[mesh_name].inc". The mesh vertices will be transformed to the frame with specified offset and
/// orientation.
ChApi bool WriteMeshPovray(const std::string& obj_filename,
                           const std::string& mesh_name,
                           const std::string& out_dir,
                           const ChColor& color = ChColor(0.4f, 0.4f, 0.4f),
                           const ChVector3d& pos = ChVector3d(0, 0, 0),
                           const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0));

/// Write the specified Bezier curve as a macro in a PovRay include file.
ChApi void WriteCurvePovray(const ChBezierCurve& curve,
                            const std::string& curve_name,
                            const std::string& out_dir,
                            double radius = 0.03,
                            const ChColor& col = ChColor(0.8f, 0.8f, 0.2f));

/// @} chrono_io

}  // namespace utils
}  // namespace chrono

#endif
