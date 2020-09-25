// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// utils file translating Chrono assets to higher quality assets
//
// =============================================================================

#ifndef CHVISUALMATERIALUTILS_H
#define CHVISUALMATERIALUTILS_H

#include <string>
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_utils
/// @{

/// Generate a set of visual assets that works with OptiX and handles the most modern rendering information contained in
/// the mesh as possible. This includes information such as materials, normal mapping, etc that Chrono traditional
/// wouldn't look for. Assets will be added directly to the mesh shape.
/// @param mesh_shape The ChTriangleMeshShape that contains a mesh for which we want to look up its additional
/// information if any exist
void CreateModernMeshAssets(std::shared_ptr<ChTriangleMeshShape> mesh_shape);

/// Convert assets attached to a ChBody to modern assets by creating all necessary parameters for reflections,
/// refractions, etc
/// @param body The body which should be parsed for old assets
void ConvertToModernAssets(std::shared_ptr<ChBody> body);

/// Parse all assets of a chrono system to do a full conversion to assets needed by OptiX
/// @param ChSystem A pointer to the Chrono system that should be parsed for old assets.
void ConvertToModernAssets(ChSystem* sys);

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
