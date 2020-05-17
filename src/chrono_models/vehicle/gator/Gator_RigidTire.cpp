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
// Gator rigid tire subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/gator/Gator_RigidTire.h"

namespace chrono {
namespace vehicle {
namespace gator {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Gator_RigidTire::m_radius = 0.467;
const double Gator_RigidTire::m_width = 0.254;

const double Gator_RigidTire::m_mass = 37.6;
const ChVector<> Gator_RigidTire::m_inertia(3.84, 6.69, 3.84);

const std::string Gator_RigidTire::m_meshFile = "gator/gator_tire_fine.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Gator_RigidTire::Gator_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    if (use_mesh) {
        SetMeshFilename(GetDataFile("gator/gator_tire_coarse.obj"), 0.005);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Gator_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

void Gator_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(vehicle::GetDataFile(m_meshFile),   // left side
                                               vehicle::GetDataFile(m_meshFile));  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void Gator_RigidTire::RemoveVisualizationAssets() {
    ChRigidTire::RemoveVisualizationAssets();
    RemoveVisualizationMesh(m_trimesh_shape);
}

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono
