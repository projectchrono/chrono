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
// Rigid tire constructed with data from file (JSON format).
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
RigidTire::RigidTire(const std::string& filename) : ChRigidTire(""), m_has_mesh(false) {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

RigidTire::RigidTire(const rapidjson::Document& d) : ChRigidTire(""), m_has_mesh(false) {
    Create(d);
}

void RigidTire::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    m_radius = d["Radius"].GetDouble();
    m_width = d["Width"].GetDouble();
    m_mass = d["Mass"].GetDouble();
    m_inertia = ReadVectorJSON(d["Inertia"]);

    // Read contact material data
    assert(d.HasMember("Contact Material"));
    m_mat_info = ReadMaterialInfoJSON(d["Contact Material"]);

    // Check if using contact mesh.
    if (d.HasMember("Contact Mesh")) {
        std::string mesh_file = d["Contact Mesh"]["Mesh Filename"].GetString();
        double sweep_radius = d["Contact Mesh"]["Sweep Sphere Radius"].GetDouble();
        SetMeshFilename(vehicle::GetDataFile(mesh_file), sweep_radius);
    }

    // Check how to visualize this tire.
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh Filename Left") && d["Visualization"].HasMember("Mesh Filename Right")) {
            m_meshFile_left = d["Visualization"]["Mesh Filename Left"].GetString();
            m_meshFile_right = d["Visualization"]["Mesh Filename Right"].GetString();
            m_has_mesh = true;
        }
    }
}

// -----------------------------------------------------------------------------
void RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    m_material = m_mat_info.CreateMaterial(contact_method);
}

void RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void RigidTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChRigidTire::RemoveVisualizationAssets();
}

}  // end namespace vehicle
}  // end namespace chrono
