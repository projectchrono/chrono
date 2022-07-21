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

const double Gator_RigidTire_Front::m_radius = 0.28575;
const double Gator_RigidTire_Front::m_width = 0.254;
const double Gator_RigidTire_Front::m_mass = 9.3;
const ChVector<> Gator_RigidTire_Front::m_inertia(0.258, 0.416, 0.258);
const std::string Gator_RigidTire_Front::m_meshFile_left = "gator/gator_wheel_FL.obj";
const std::string Gator_RigidTire_Front::m_meshFile_right = "gator/gator_wheel_FR.obj";
////const std::string Gator_RigidTire_Front::m_meshFile_left = "gator/gator_tireF_fine.obj";
////const std::string Gator_RigidTire_Front::m_meshFile_right = "gator/gator_tireF_fine.obj";

const double Gator_RigidTire_Rear::m_radius = 0.3175;
const double Gator_RigidTire_Rear::m_width = 0.3048;
const double Gator_RigidTire_Rear::m_mass = 9.3;
const ChVector<> Gator_RigidTire_Rear::m_inertia(0.258, 0.416, 0.258);
const std::string Gator_RigidTire_Rear::m_meshFile_left = "gator/gator_wheel_RL.obj";
const std::string Gator_RigidTire_Rear::m_meshFile_right = "gator/gator_wheel_RR.obj";
////const std::string Gator_RigidTire_Rear::m_meshFile_left = "gator/gator_tireR_fine.obj";
////const std::string Gator_RigidTire_Rear::m_meshFile_right = "gator/gator_tireR_fine.obj";

// -----------------------------------------------------------------------------

Gator_RigidTire_Front::Gator_RigidTire_Front(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    if (use_mesh) {
        SetMeshFilename(GetDataFile("gator/gator_tireF_coarse.obj"), 0.005);
    }
}

void Gator_RigidTire_Front::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

void Gator_RigidTire_Front::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void Gator_RigidTire_Front::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChRigidTire::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------

Gator_RigidTire_Rear::Gator_RigidTire_Rear(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    if (use_mesh) {
        SetMeshFilename(GetDataFile("gator/gator_tireR_coarse.obj"), 0.005);
    }
}

void Gator_RigidTire_Rear::CreateContactMaterial(ChContactMethod contact_method) {
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

void Gator_RigidTire_Rear::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void Gator_RigidTire_Rear::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChRigidTire::RemoveVisualizationAssets();
}

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono
