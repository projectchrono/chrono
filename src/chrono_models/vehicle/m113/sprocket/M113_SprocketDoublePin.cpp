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
// M113 sprocket subsystem (double pin).
//
// =============================================================================

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/sprocket/M113_SprocketDoublePin.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const int M113_SprocketDoublePin::m_num_teeth = 10;

const double M113_SprocketDoublePin::m_gear_mass = 27.68;
const ChVector<> M113_SprocketDoublePin::m_gear_inertia(0.646, 0.883, 0.646);
const double M113_SprocketDoublePin::m_axle_inertia = 0.4;
const double M113_SprocketDoublePin::m_separation = 0.278;

const double M113_SprocketDoublePin::m_gear_RT = 0.2715;  // 10.69''
const double M113_SprocketDoublePin::m_gear_R = 0.0223;   // 0.88''
const double M113_SprocketDoublePin::m_gear_RA = 0.242;   // 9.53''

const double M113_SprocketDoublePin::m_gear_C = 0.2371;  // 9.334''
const double M113_SprocketDoublePin::m_gear_W = 0.0464;  // 1.825''

const double M113_SprocketDoublePin::m_lateral_backlash = 0.02;

const std::string M113_SprocketDoublePinLeft::m_meshFile = "M113/meshes/Sprocket2_L.obj";
const std::string M113_SprocketDoublePinRight::m_meshFile = "M113/meshes/Sprocket2_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_SprocketDoublePin::M113_SprocketDoublePin(const std::string& name) : ChSprocketDoublePin(name) {}

void M113_SprocketDoublePin::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.4f;
    minfo.cr = 0.75f;
    minfo.Y = 1e9f;
    m_material = minfo.CreateMaterial(contact_method);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_SprocketDoublePin::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        ////auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(GetMeshFile(), false, false);
        auto trimesh = CreateVisualizationMesh(0.15, 0.03, 0.02);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(GetMeshFile()).stem());
        trimesh_shape->SetMutable(false);
        ////std::vector<geometry::ChTriangleMeshConnected> meshes = {*trimesh};
        ////geometry::ChTriangleMeshConnected::WriteWavefront("mySprocket.obj", meshes);
        m_gear->AddVisualShape(trimesh_shape);
    } else {
        ChSprocket::AddVisualizationAssets(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
