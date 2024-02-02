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
// Generic vehicle rigid tire subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/generic/tire/Generic_RigidTire.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Generic_RigidTire::m_radius = 0.467;
const double Generic_RigidTire::m_width = 0.254;

const double Generic_RigidTire::m_mass = 37.6;
const ChVector<> Generic_RigidTire::m_inertia(3.84, 6.69, 3.84);

const std::string Generic_RigidTire::m_meshFile_left = "generic/tire/generic_tire_coarse.obj";
const std::string Generic_RigidTire::m_meshFile_right = "generic/tire/generic_tire_coarse.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Generic_RigidTire::Generic_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    if (use_mesh) {
        SetMeshFilename(GetDataFile("generic/tire/generic_tire_coarse.obj"), 0.005);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Generic_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

void Generic_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void Generic_RigidTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChRigidTire::RemoveVisualizationAssets();
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
