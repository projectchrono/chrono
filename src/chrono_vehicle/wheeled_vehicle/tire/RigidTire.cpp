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
    Document d = ReadFileJSON(filename);
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

    // Load default values (in case not all are provided in the JSON file)
    m_mat_info.mu = 0.9f;
    m_mat_info.cr = 0.01f;
    m_mat_info.Y = 2e7f;
    m_mat_info.nu = 0.3f;
    m_mat_info.kn = 2e5f;
    m_mat_info.gn = 40.0f;
    m_mat_info.kt = 2e5f;
    m_mat_info.gt = 20.0f;

    const Value& mat = d["Contact Material"];

    m_mat_info.mu = mat["Coefficient of Friction"].GetFloat();
    m_mat_info.cr = mat["Coefficient of Restitution"].GetFloat();
    if (mat.HasMember("Properties")) {
        m_mat_info.Y = mat["Properties"]["Young Modulus"].GetFloat();
        m_mat_info.nu = mat["Properties"]["Poisson Ratio"].GetFloat();
    }
    if (mat.HasMember("Coefficients")) {
        m_mat_info.kn = mat["Coefficients"]["Normal Stiffness"].GetFloat();
        m_mat_info.gn = mat["Coefficients"]["Normal Damping"].GetFloat();
        m_mat_info.kt = mat["Coefficients"]["Tangential Stiffness"].GetFloat();
        m_mat_info.gt = mat["Coefficients"]["Tangential Damping"].GetFloat();
    }

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
    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(m_mat_info.mu);
            matNSC->SetRestitution(m_mat_info.cr);
            m_material = matNSC;
            break;
        }
        case ChContactMethod::SMC:
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(m_mat_info.mu);
            matSMC->SetRestitution(m_mat_info.cr);
            matSMC->SetYoungModulus(m_mat_info.Y);
            matSMC->SetPoissonRatio(m_mat_info.nu);
            matSMC->SetKn(m_mat_info.kn);
            matSMC->SetGn(m_mat_info.gn);
            matSMC->SetKt(m_mat_info.kt);
            matSMC->SetGt(m_mat_info.gt);
            m_material = matSMC;
            break;
    }
}

void RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        m_trimesh_shape = AddVisualizationMesh(vehicle::GetDataFile(m_meshFile_left),    // left side
                                               vehicle::GetDataFile(m_meshFile_right));  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void RigidTire::RemoveVisualizationAssets() {
    ChRigidTire::RemoveVisualizationAssets();
    RemoveVisualizationMesh(m_trimesh_shape);
}

}  // end namespace vehicle
}  // end namespace chrono
