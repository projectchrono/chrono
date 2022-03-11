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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// FMTV rigid tire subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/mtv/FMTV_RigidTire.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double FMTV_RigidTire::m_radius = 0.531;
const double FMTV_RigidTire::m_width = 0.395;

const double FMTV_RigidTire::m_mass = 110.0;
const ChVector<> FMTV_RigidTire::m_inertia(13.0, 22.0, 13.0);

const std::string FMTV_RigidTire::m_meshFile = "mtv/meshes/fmtv_tire_fine.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FMTV_RigidTire::FMTV_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    if (use_mesh) {
        SetMeshFilename(GetDataFile("mtv/meshes/mtv_tire_fine.obj"), 0.005);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void FMTV_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

void FMTV_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void FMTV_RigidTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChRigidTire::RemoveVisualizationAssets();
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
