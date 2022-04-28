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
// M113 sprocket subsystem (single pin).
//
// =============================================================================

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/M113_SprocketSinglePin.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const int M113_SprocketSinglePin::m_num_teeth = 10;

const double M113_SprocketSinglePin::m_gear_mass = 27.68;
const ChVector<> M113_SprocketSinglePin::m_gear_inertia(0.646, 0.883, 0.646);
const double M113_SprocketSinglePin::m_axle_inertia = 0.4;
const double M113_SprocketSinglePin::m_separation = 0.225;

const double M113_SprocketSinglePin::m_gear_RT = 0.2605;
const double M113_SprocketSinglePin::m_gear_RC = 0.3;
const double M113_SprocketSinglePin::m_gear_R = 0.089;
const double M113_SprocketSinglePin::m_gear_RA = 0.245;

const double M113_SprocketSinglePin::m_lateral_backlash = 0.02;

const std::string M113_SprocketSinglePinLeft::m_meshFile = "M113/meshes/Sprocket_L.obj";
const std::string M113_SprocketSinglePinRight::m_meshFile = "M113/meshes/Sprocket_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_SprocketSinglePin::M113_SprocketSinglePin(const std::string& name) : ChSprocketSinglePin(name) {}

void M113_SprocketSinglePin::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.4f;
    minfo.cr = 0.75f;
    minfo.Y = 1e9f;
    m_material = minfo.CreateMaterial(contact_method);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_SprocketSinglePin::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(GetMeshFile(), false, false);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(GetMeshFile()).stem());
        trimesh_shape->SetMutable(false);
        m_gear->AddVisualShape(trimesh_shape);
    } else {
        ChSprocket::AddVisualizationAssets(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
