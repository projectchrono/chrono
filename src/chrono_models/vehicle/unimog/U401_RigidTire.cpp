// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// U401 rigid tire subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/unimog/U401_RigidTire.h"

namespace chrono {
namespace vehicle {
namespace unimog {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double U401_RigidTire::m_radius = 0.4375;
const double U401_RigidTire::m_width = 0.184;

const double U401_RigidTire::m_mass = 28.0;
const ChVector<> U401_RigidTire::m_inertia(1.65, 2.90, 1.65);

const std::string U401_RigidTire::m_meshFile = "unimog/U401_Tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
U401_RigidTire::U401_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    if (use_mesh) {
        SetMeshFilename(GetDataFile("unimog/U401_Tire.obj"), 0.005);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void U401_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

void U401_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void U401_RigidTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChRigidTire::RemoveVisualizationAssets();
}

}  // namespace unimog
}  // end namespace vehicle
}  // end namespace chrono
