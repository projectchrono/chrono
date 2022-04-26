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
// HMMWV rigid tire subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_RigidTire.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double HMMWV_RigidTire::m_radius = 0.467;
const double HMMWV_RigidTire::m_width = 0.254;

const double HMMWV_RigidTire::m_mass = 37.6;
const ChVector<> HMMWV_RigidTire::m_inertia(3.84, 6.69, 3.84);

const std::string HMMWV_RigidTire::m_meshFile_left = "hmmwv/hmmwv_tire_left.obj";
const std::string HMMWV_RigidTire::m_meshFile_right = "hmmwv/hmmwv_tire_right.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_RigidTire::HMMWV_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    if (use_mesh) {
        SetMeshFilename(GetDataFile("hmmwv/hmmwv_tire_coarse.obj"), 0.005);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

void HMMWV_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void HMMWV_RigidTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChRigidTire::RemoveVisualizationAssets();
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
