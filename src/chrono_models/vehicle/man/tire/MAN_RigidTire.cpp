// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// MAN rigid tire subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_models/vehicle/man/tire/MAN_RigidTire.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double MAN_RigidTire::m_radius = 0.629;
const double MAN_RigidTire::m_width = 0.385;

const double MAN_RigidTire::m_mass = 104.0;
const ChVector3d MAN_RigidTire::m_inertia(17.86, 31.66, 17.86);

const std::string MAN_RigidTire::m_meshFile_left = "MAN/meshes/MAN_tire.obj";
const std::string MAN_RigidTire::m_meshFile_right = "MAN/meshes/MAN_tire.obj";

// -----------------------------------------------------------------------------

MAN_RigidTire::MAN_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    if (use_mesh) {
        SetContactMesh(GetVehicleDataFile("MAN/meshes/MAN_tire_collision_mesh.obj"), 0.005);
    }
}

void MAN_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

void MAN_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void MAN_RigidTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChRigidTire::RemoveVisualizationAssets();
}

}  // end namespace man
}  // end namespace vehicle
}  // end namespace chrono
