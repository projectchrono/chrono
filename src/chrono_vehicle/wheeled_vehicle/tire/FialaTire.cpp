// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Michael Taylor, Rainer Gericke
// =============================================================================
//
// Fiala tire constructed with data from file (JSON format).
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FialaTire::FialaTire(const std::string& filename) : ChFialaTire(""), m_has_mesh(false) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

FialaTire::FialaTire(const rapidjson::Document& d) : ChFialaTire(""), m_has_mesh(false) {
    Create(d);
}

FialaTire::~FialaTire() {}

void FialaTire::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    m_mass = d["Mass"].GetDouble();
    m_inertia = ReadVectorJSON(d["Inertia"]);
    if (d.HasMember("Coefficient of Friction")) {
        // Default value = 0.8
        m_mu_0 = d["Coefficient of Friction"].GetDouble();
    }
    // Read in Fiala tire model parameters
    m_unloaded_radius = d["Fiala Parameters"]["Unloaded Radius"].GetDouble();
    m_width = d["Fiala Parameters"]["Width"].GetDouble();
    m_normalStiffness = d["Fiala Parameters"]["Vertical Stiffness"].GetDouble();
    m_normalDamping = d["Fiala Parameters"]["Vertical Damping"].GetDouble();
    m_rolling_resistance = d["Fiala Parameters"]["Rolling Resistance"].GetDouble();
    m_c_slip = d["Fiala Parameters"]["CSLIP"].GetDouble();
    m_c_alpha = d["Fiala Parameters"]["CALPHA"].GetDouble();
    m_u_min = d["Fiala Parameters"]["UMIN"].GetDouble();
    m_u_max = d["Fiala Parameters"]["UMAX"].GetDouble();
    m_relax_length_x = d["Fiala Parameters"]["X Relaxation Length"].GetDouble();
    m_relax_length_y = d["Fiala Parameters"]["Y Relaxation Length"].GetDouble();
    if (m_relax_length_x <= 0.0 || m_relax_length_y <= 0.0) {
        m_dynamic_mode = false;
    }
    m_visualization_width = m_width;

    // Check how to visualize this tire.
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh Filename")) {
            m_meshFile = d["Visualization"]["Mesh Filename"].GetString();
            m_meshName = d["Visualization"]["Mesh Name"].GetString();
            m_has_mesh = true;
        }

        if (d["Visualization"].HasMember("Width")) {
            m_visualization_width = d["Visualization"]["Width"].GetDouble();
        }
    }
}

// -----------------------------------------------------------------------------
void FialaTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_trimesh_shape->SetStatic(true);
        m_wheel->AddAsset(m_trimesh_shape);
    } else {
        ChFialaTire::AddVisualizationAssets(vis);
    }
}

void FialaTire::RemoveVisualizationAssets() {
    ChFialaTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by FialaTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_trimesh_shape);
    if (it != m_wheel->GetAssets().end())
        m_wheel->GetAssets().erase(it);
}

}  // end namespace vehicle
}  // end namespace chrono
