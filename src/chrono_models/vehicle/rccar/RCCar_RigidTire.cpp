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

const double RCCar_RigidTire::m_radius = .170/2;
const double RCCar_RigidTire::m_width = .0855;

const double RCCar_RigidTire::m_mass = .200;
const ChVector<> RCCar_RigidTire::m_inertia(.0008, 0.001, .0008);

const std::string RCCar_RigidTire::m_meshName = "RCCar_tire_POV_geom";
//const std::string RCCar_RigidTire::m_meshFile = "rccar/tire.obj";

const std::string RCCar_RigidTire::m_meshFile_left = "rccar/rccar_tire_left.obj";
const std::string RCCar_RigidTire::m_meshFile_right = "rccar/rccar_tire_left.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void RCCar_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.9f; //TODO
    minfo.cr = 0.1f; //TODO
    minfo.Y = 2e7f; //TODO
    m_material = minfo.CreateMaterial(contact_method);
}

RCCar_RigidTire::RCCar_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RCCar_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
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
