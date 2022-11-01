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
// FEDA rigid tire subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/feda/FEDA_RigidTire.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double FEDA_RigidTire::m_radius = 0.499;
const double FEDA_RigidTire::m_width = 0.335;

const double FEDA_RigidTire::m_mass = 55.4;
const ChVector<> FEDA_RigidTire::m_inertia(6.39, 11.31, 6.39);

const std::string FEDA_RigidTire::m_meshFile = "feda/meshes/feda_tire_fine.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FEDA_RigidTire::FEDA_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    if (use_mesh) {
        SetMeshFilename(GetDataFile("feda/meshes/feda_tire_fine.obj"), 0.005);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void FEDA_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

void FEDA_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void FEDA_RigidTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChRigidTire::RemoveVisualizationAssets();
}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono
