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
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// LMTV rigid tire subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/mtv/LMTV_RigidTire.h"

namespace chrono {
namespace vehicle {
namespace mtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double LMTV_RigidTire::m_radius = 0.531;
const double LMTV_RigidTire::m_width = 0.395;

const double LMTV_RigidTire::m_mass = 110.0;
const ChVector<> LMTV_RigidTire::m_inertia(13.0, 22.0, 13.0);

const std::string LMTV_RigidTire::m_meshFile = "lmtv/meshes/lmtv_tire_fine.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
LMTV_RigidTire::LMTV_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    if (use_mesh) {
        SetMeshFilename(GetDataFile("lmtv/meshes/lmtv_tire_fine.obj"), 0.005);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void LMTV_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

void LMTV_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(vehicle::GetDataFile(m_meshFile),   // left side
                                               vehicle::GetDataFile(m_meshFile));  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void LMTV_RigidTire::RemoveVisualizationAssets() {
    ChRigidTire::RemoveVisualizationAssets();
    RemoveVisualizationMesh(m_trimesh_shape);
}

}  // namespace mtv
}  // end namespace vehicle
}  // end namespace chrono
