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

#include "chrono_models/vehicle/m113a/M113a_SprocketSinglePin.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const int M113a_SprocketSinglePin::m_num_teeth = 10;

const double M113a_SprocketSinglePin::m_gear_mass = 65.422;
const ChVector<> M113a_SprocketSinglePin::m_gear_inertia(1.053, 1.498, 1.053);
const double M113a_SprocketSinglePin::m_axle_inertia = 1;
const double M113a_SprocketSinglePin::m_separation = 0.21082;

const double M113a_SprocketSinglePin::m_gear_RT = 0.2605;
const double M113a_SprocketSinglePin::m_gear_RC = 0.3;
const double M113a_SprocketSinglePin::m_gear_R = 0.089;
const double M113a_SprocketSinglePin::m_gear_RA = 0.245;

const std::string M113a_SprocketSinglePinLeft::m_meshFile = "M113/Sprocket_L.obj";
const std::string M113a_SprocketSinglePinRight::m_meshFile = "M113/Sprocket_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113a_SprocketSinglePin::M113a_SprocketSinglePin(const std::string& name) : ChSprocketSinglePin(name) {}

void M113a_SprocketSinglePin::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.4f;
    minfo.cr = 0.1f;
    minfo.Y = 1e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113a_SprocketSinglePin::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(GetMeshFile(), false, false);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(GetMeshFile()).stem());
        trimesh_shape->SetStatic(true);
        m_gear->AddAsset(trimesh_shape);
    } else {
        ChSprocket::AddVisualizationAssets(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
