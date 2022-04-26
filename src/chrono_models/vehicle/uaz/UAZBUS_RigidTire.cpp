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
// UAZBUS rigid tire subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/uaz/UAZBUS_RigidTire.h"

namespace chrono {
namespace vehicle {
namespace uaz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double UAZBUS_RigidTire::m_radius = 0.372;
const double UAZBUS_RigidTire::m_width = 0.228;

const double UAZBUS_RigidTire::m_mass = 19.8;
const ChVector<> UAZBUS_RigidTire::m_inertia(1.2369, 2.22357, 1.2369);

const std::string UAZBUS_RigidTire::m_meshFile = "uaz/uaz_tire_fine.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
UAZBUS_RigidTire::UAZBUS_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    if (use_mesh) {
        SetMeshFilename(GetDataFile("uaz/uaz_tire_fine.obj"), 0.005);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void UAZBUS_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

void UAZBUS_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void UAZBUS_RigidTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChRigidTire::RemoveVisualizationAssets();
}

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
