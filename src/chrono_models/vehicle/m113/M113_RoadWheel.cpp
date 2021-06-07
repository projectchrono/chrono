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
// M113 road wheel subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/M113_RoadWheel.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_RoadWheel::m_wheel_mass = 35.56;
const ChVector<> M113_RoadWheel::m_wheel_inertia(1.14, 2.16, 1.14);
const double M113_RoadWheel::m_wheel_radius = 0.305;
const double M113_RoadWheel::m_wheel_width = 0.181;
const double M113_RoadWheel::m_wheel_gap = 0.051;

const std::string M113_RoadWheelLeft::m_meshFile = "M113/Roller_L.obj";
const std::string M113_RoadWheelRight::m_meshFile = "M113/Roller_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_RoadWheel::M113_RoadWheel(const std::string& name) : ChDoubleRoadWheel(name) {}

void M113_RoadWheel::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.4f;
    minfo.cr = 0.75f;
    minfo.Y = 1e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_RoadWheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(GetMeshFile(), false, false);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(GetMeshFile()).stem());
        trimesh_shape->SetStatic(true);
        m_wheel->AddAsset(trimesh_shape);
    } else {
        ChDoubleRoadWheel::AddVisualizationAssets(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
