// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
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
// M113 idler wheel subsystem.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/M113_IdlerWheel.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_IdlerWheel::m_wheel_mass = 25.76;
const ChVector<> M113_IdlerWheel::m_wheel_inertia(0.618, 1.12, 0.618);
const double M113_IdlerWheel::m_wheel_radius = 0.255;
const double M113_IdlerWheel::m_wheel_width = 0.181;
const double M113_IdlerWheel::m_wheel_gap = 0.051;

const std::string M113_IdlerWheelLeft::m_meshFile = "M113/meshes/Idler_L.obj";
const std::string M113_IdlerWheelRight::m_meshFile = "M113/meshes/Idler_R.obj";

// -----------------------------------------------------------------------------

M113_IdlerWheel::M113_IdlerWheel(const std::string& name) : ChDoubleTrackWheel(name) {}

void M113_IdlerWheel::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.4f;
    minfo.cr = 0.75f;
    minfo.Y = 1e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

void M113_IdlerWheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(GetMeshFile(), false, false);
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(GetMeshFile()).stem());
        trimesh_shape->SetMutable(false);
        m_wheel->AddVisualShape(trimesh_shape);
    } else {
        ChDoubleTrackWheel::AddVisualizationAssets(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
