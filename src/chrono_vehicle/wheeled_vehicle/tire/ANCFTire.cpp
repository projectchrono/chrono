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
// ANCF tire constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/core/ChCubicSpline.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace chrono::fea;
using namespace rapidjson;

namespace chrono {
namespace vehicle {

// Constructors for ANCFTire
ANCFTire::ANCFTire(const std::string& filename) : ChANCFTire(""), m_ANCF8(false) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    ProcessJSON(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ANCFTire::ANCFTire(const rapidjson::Document& d) : ChANCFTire("") {
    ProcessJSON(d);
}

// Process the specified JSON document and load tire specification
void ANCFTire::ProcessJSON(const rapidjson::Document& d) {
    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    SetName(d["Name"].GetString());

    // Read geometric dimensions
    m_tire_radius = d["Tire Radius"].GetDouble();
    m_rim_radius = d["Rim Radius"].GetDouble();
    m_rim_width = d["Rim Width"].GetDouble();

    if (d.HasMember("8-Node Elements"))
        m_ANCF8 = d["8-Node Elements"].GetBool();

    // Read contact material data
    assert(d.HasMember("Contact Material"));
    m_mat_info = ReadMaterialInfoJSON(d["Contact Material"]);

    // Read the list of materials (note that order is important)
    int num_materials = d["Materials"].Size();
    m_materials.resize(num_materials);
    for (int i = 0; i < num_materials; i++) {
        std::string type = d["Materials"][i]["Type"].GetString();
        if (type.compare("Isotropic") == 0) {
            double rho = d["Materials"][i]["Density"].GetDouble();
            double E = d["Materials"][i]["E"].GetDouble();
            double nu = d["Materials"][i]["nu"].GetDouble();
            m_materials[i] = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu);
        } else if (type.compare("Orthotropic") == 0) {
            double rho = d["Materials"][i]["Density"].GetDouble();
            ChVector<> E = ReadVectorJSON(d["Materials"][i]["E"]);
            ChVector<> nu = ReadVectorJSON(d["Materials"][i]["nu"]);
            ChVector<> G = ReadVectorJSON(d["Materials"][i]["G"]);
            m_materials[i] = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu, G);
        }
    }

    // Structural damping
    m_alpha = d["Structural Damping Coefficient"].GetDouble();

    // Default tire pressure
    m_default_pressure = d["Default Pressure"].GetDouble();

    // Read layer information for the Bead Section
    m_bead.num_divs = d["Bead Section"]["Number Elements"].GetInt();
    m_bead.num_layers = d["Bead Section"]["Layer Thickness"].Size();
    assert(d["Bead Section"]["Ply Angle"].Size() == m_bead.num_layers);
    assert(d["Bead Section"]["Material ID"].Size() == m_bead.num_layers);
    for (int i = 0; i < m_bead.num_layers; i++) {
        double thickness = d["Bead Section"]["Layer Thickness"][i].GetDouble();
        double angle = d["Bead Section"]["Ply Angle"][i].GetDouble();
        int id = d["Bead Section"]["Material ID"][i].GetInt();
        assert(id >= 0 && id < num_materials);
        m_bead.thickness.push_back(thickness);
        m_bead.angle.push_back(angle);
        m_bead.mat.push_back(m_materials[id]);
    }

    // Read layer information for the Sidewall Section
    m_sidewall.num_divs = d["Sidewall Section"]["Number Elements"].GetInt();
    m_sidewall.num_layers = d["Sidewall Section"]["Layer Thickness"].Size();
    assert(d["Sidewall Section"]["Ply Angle"].Size() == m_sidewall.num_layers);
    assert(d["Sidewall Section"]["Material ID"].Size() == m_sidewall.num_layers);
    for (int i = 0; i < m_sidewall.num_layers; i++) {
        double thickness = d["Sidewall Section"]["Layer Thickness"][i].GetDouble();
        double angle = d["Sidewall Section"]["Ply Angle"][i].GetDouble();
        int id = d["Sidewall Section"]["Material ID"][i].GetInt();
        assert(id >= 0 && id < num_materials);
        m_sidewall.thickness.push_back(thickness);
        m_sidewall.angle.push_back(angle);
        m_sidewall.mat.push_back(m_materials[id]);
    }

    // Read layer information for the Tread Section
    m_tread.num_divs = d["Tread Section"]["Number Elements"].GetInt();
    m_tread.num_layers = d["Tread Section"]["Layer Thickness"].Size();
    assert(d["Tread Section"]["Ply Angle"].Size() == m_tread.num_layers);
    assert(d["Tread Section"]["Material ID"].Size() == m_tread.num_layers);
    for (int i = 0; i < m_tread.num_layers; i++) {
        double thickness = d["Tread Section"]["Layer Thickness"][i].GetDouble();
        double angle = d["Tread Section"]["Ply Angle"][i].GetDouble();
        int id = d["Tread Section"]["Material ID"][i].GetInt();
        assert(id >= 0 && id < num_materials);
        m_tread.thickness.push_back(thickness);
        m_tread.angle.push_back(angle);
        m_tread.mat.push_back(m_materials[id]);
    }

    // Number of elements in the two orthogonal directions
    m_div_circumference = d["Number Elements Circumference"].GetInt();

    // Read profile specification
    m_num_points = d["Profile"].Size();
    m_profile_t.resize(m_num_points);
    m_profile_x.resize(m_num_points);
    m_profile_y.resize(m_num_points);
    for (unsigned int i = 0; i < m_num_points; i++) {
        m_profile_t[i] = d["Profile"][i][0u].GetDouble();
        m_profile_x[i] = d["Profile"][i][1u].GetDouble();
        m_profile_y[i] = d["Profile"][i][2u].GetDouble();
    }
}

// Create the FEA mesh
void ANCFTire::CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) {
    if (m_ANCF8)
        m_rim_nodes = CreateMeshANCF8(                //
            {m_profile_t, m_profile_x, m_profile_y},  //
            m_bead, m_sidewall, m_tread,              //
            m_div_circumference,                      //
            m_rim_radius,                             //
            m_alpha,                                  //
            m_mesh,                                   //
            wheel_frame                               //
        );
    else
        m_rim_nodes = CreateMeshANCF4(                //
            {m_profile_t, m_profile_x, m_profile_y},  //
            m_bead, m_sidewall, m_tread,              //
            m_div_circumference,                      //
            m_rim_radius,                             //
            m_alpha,                                  //
            m_mesh,                                   //
            wheel_frame                               //
        );
}

void ANCFTire::CreateContactMaterial() {
    m_contact_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    m_contact_mat->SetFriction(m_mat_info.mu);
    m_contact_mat->SetRestitution(m_mat_info.cr);
    m_contact_mat->SetYoungModulus(m_mat_info.Y);
    m_contact_mat->SetPoissonRatio(m_mat_info.nu);
    m_contact_mat->SetKn(m_mat_info.kn);
    m_contact_mat->SetGn(m_mat_info.gn);
    m_contact_mat->SetKt(m_mat_info.kt);
    m_contact_mat->SetGt(m_mat_info.gt);
}

}  // end namespace vehicle
}  // end namespace chrono
