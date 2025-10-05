// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// Jeep Cherokee rigid tire model based on ChShaft objects.
// Vehicle Parameters taken from SAE Paper 1999-01-0121
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_models/vehicle/jeep/Cherokee_RigidTire.h"

namespace chrono {
namespace vehicle {
namespace jeep {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Cherokee_RigidTire::m_radius = 0.3665;
const double Cherokee_RigidTire::m_width = 0.225;

const double Cherokee_RigidTire::m_mass =  13.78;
const ChVector3d Cherokee_RigidTire::m_inertia(0.8335, 1.5513, 0.8335);

const std::string Cherokee_RigidTire::m_meshFile_left = "jeep/Cherokee_Tire.obj";
const std::string Cherokee_RigidTire::m_meshFile_right = "jeep/Cherokee_Tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Cherokee_RigidTire::Cherokee_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    if (use_mesh) {
        SetMeshFilename(GetVehicleDataFile("hmmwv/hmmwv_tire_coarse.obj"), 0.005);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Cherokee_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

void Cherokee_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void Cherokee_RigidTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChRigidTire::RemoveVisualizationAssets();
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
