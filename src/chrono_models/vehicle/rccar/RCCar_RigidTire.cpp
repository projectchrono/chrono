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
// Authors: Radu Serban, Jayne Henry
// =============================================================================
//
// RCCar rigid tire subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/rccar/RCCar_RigidTire.h"

using namespace chrono::vehicle;
using namespace chrono;

namespace chrono {
namespace vehicle {
namespace rccar {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;

const double RCCar_RigidTire::m_radius = in2m * 3.285;
const double RCCar_RigidTire::m_width = in2m * 3.3;

const double RCCar_RigidTire::m_mass = lb2kg * 1;
const ChVector<> RCCar_RigidTire::m_inertia(0.001553, 0.002521, 0.001553);

const std::string RCCar_RigidTire::m_meshName = "RCCar_tire_POV_geom";
// const std::string RCCar_RigidTire::m_meshFile = "rccar/RCCar_tire.obj";
const std::string RCCar_RigidTire::m_meshFile = "rccar/tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void RCCar_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

RCCar_RigidTire::RCCar_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    if (use_mesh) {
        // SetMeshFilename(GetDataFile("rccar/RCCar_tire.obj"), 0.005);
        SetMeshFilename(GetDataFile(m_meshFile), 0.005);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RCCar_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_trimesh_shape->SetStatic(true);
        m_wheel->GetSpindle()->AddAsset(m_trimesh_shape);
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void RCCar_RigidTire::RemoveVisualizationAssets() {
    ChRigidTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by RCCar_RigidTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto& assets = m_wheel->GetSpindle()->GetAssets();
    auto it = std::find(assets.begin(), assets.end(), m_trimesh_shape);
    if (it != assets.end())
        assets.erase(it);
}

}  // end namespace rccar
}  // namespace vehicle
}  // namespace chrono
