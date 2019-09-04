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
// Pac89 tire constructed with data from file (JSON format).
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/tire/Pac89Tire.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Pac89Tire::Pac89Tire(const std::string& filename) : ChPac89Tire(""), m_mass(0), m_normalDamping(0), m_has_mesh(false) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

Pac89Tire::Pac89Tire(const rapidjson::Document& d) : ChPac89Tire(""), m_mass(0), m_normalDamping(0), m_has_mesh(false) {
    Create(d);
}

void Pac89Tire::Create(const rapidjson::Document& d) {  // Invoke base class method.
    ChPart::Create(d);

    m_has_vert_table = false;
    m_mass = d["Mass"].GetDouble();
    GetLog() << "Masse = " << m_mass << "\n";
    m_inertia = ReadVectorJSON(d["Inertia"]);
    if (d.HasMember("Coefficient of Friction")) {
        // Default value = 0.8
        m_mu0 = d["Coefficient of Friction"].GetDouble();
    }
    if (d.HasMember("Design Parameters")) {
        // Read in Pac89 design tire model parameters
        m_unloaded_radius = d["Design Parameters"]["Unloaded Radius"].GetDouble();
        m_width = d["Design Parameters"]["Width"].GetDouble();
        m_lateral_stiffness = d["Design Parameters"]["Lateral Stiffness"].GetDouble();
        m_normalStiffness = d["Design Parameters"]["Vertical Stiffness"].GetDouble();
        if (d["Design Parameters"].HasMember("Vertical Curve Data")) {
            int num_points = d["Design Parameters"]["Vertical Curve Data"].Size();
            for (int i = 0; i < num_points; i++) {
                m_vert_map.AddPoint(d["Design Parameters"]["Vertical Curve Data"][i][0u].GetDouble(),
                                    d["Design Parameters"]["Vertical Curve Data"][i][1u].GetDouble());
            }
            m_has_vert_table = true;
            GetLog() << "numPts = " << num_points << "\n";
        }
        m_normalDamping = d["Design Parameters"]["Vertical Damping"].GetDouble();
        m_rolling_resistance = d["Design Parameters"]["Rolling Resistance"].GetDouble();
    }
    if (d.HasMember("Lateral Coefficients")) {
        m_PacCoeff.A0 = d["Lateral Coefficients"]["a0"].GetDouble();
        m_PacCoeff.A1 = d["Lateral Coefficients"]["a1"].GetDouble();
        m_PacCoeff.A2 = d["Lateral Coefficients"]["a2"].GetDouble();
        m_PacCoeff.A3 = d["Lateral Coefficients"]["a3"].GetDouble();
        m_PacCoeff.A4 = d["Lateral Coefficients"]["a4"].GetDouble();
        m_PacCoeff.A5 = d["Lateral Coefficients"]["a5"].GetDouble();
        m_PacCoeff.A6 = d["Lateral Coefficients"]["a6"].GetDouble();
        m_PacCoeff.A7 = d["Lateral Coefficients"]["a7"].GetDouble();
        m_PacCoeff.A8 = d["Lateral Coefficients"]["a8"].GetDouble();
        m_PacCoeff.A9 = d["Lateral Coefficients"]["a9"].GetDouble();
        m_PacCoeff.A10 = d["Lateral Coefficients"]["a10"].GetDouble();
        m_PacCoeff.A11 = d["Lateral Coefficients"]["a11"].GetDouble();
        m_PacCoeff.A12 = d["Lateral Coefficients"]["a12"].GetDouble();
        GetLog() << "A0 = " << m_PacCoeff.A0 << "\n";
    }
    if (d.HasMember("Longitudinal Coefficients")) {
        m_PacCoeff.B0 = d["Longitudinal Coefficients"]["b0"].GetDouble();
        m_PacCoeff.B1 = d["Longitudinal Coefficients"]["b1"].GetDouble();
        m_PacCoeff.B2 = d["Longitudinal Coefficients"]["b2"].GetDouble();
        m_PacCoeff.B3 = d["Longitudinal Coefficients"]["b3"].GetDouble();
        m_PacCoeff.B4 = d["Longitudinal Coefficients"]["b4"].GetDouble();
        m_PacCoeff.B5 = d["Longitudinal Coefficients"]["b5"].GetDouble();
        m_PacCoeff.B6 = d["Longitudinal Coefficients"]["b6"].GetDouble();
        m_PacCoeff.B7 = d["Longitudinal Coefficients"]["b7"].GetDouble();
        m_PacCoeff.B8 = d["Longitudinal Coefficients"]["b8"].GetDouble();
        m_PacCoeff.B9 = d["Longitudinal Coefficients"]["b9"].GetDouble();
        m_PacCoeff.B10 = d["Longitudinal Coefficients"]["b10"].GetDouble();
    }
    if (d.HasMember("Aligning Coefficients")) {
        m_PacCoeff.C0 = d["Aligning Coefficients"]["c0"].GetDouble();
        m_PacCoeff.C1 = d["Aligning Coefficients"]["c1"].GetDouble();
        m_PacCoeff.C2 = d["Aligning Coefficients"]["c2"].GetDouble();
        m_PacCoeff.C3 = d["Aligning Coefficients"]["c3"].GetDouble();
        m_PacCoeff.C4 = d["Aligning Coefficients"]["c4"].GetDouble();
        m_PacCoeff.C5 = d["Aligning Coefficients"]["c5"].GetDouble();
        m_PacCoeff.C6 = d["Aligning Coefficients"]["c6"].GetDouble();
        m_PacCoeff.C7 = d["Aligning Coefficients"]["c7"].GetDouble();
        m_PacCoeff.C8 = d["Aligning Coefficients"]["c8"].GetDouble();
        m_PacCoeff.C9 = d["Aligning Coefficients"]["c9"].GetDouble();
        m_PacCoeff.C10 = d["Aligning Coefficients"]["c10"].GetDouble();
        m_PacCoeff.C11 = d["Aligning Coefficients"]["c11"].GetDouble();
        m_PacCoeff.C12 = d["Aligning Coefficients"]["c12"].GetDouble();
        m_PacCoeff.C13 = d["Aligning Coefficients"]["c13"].GetDouble();
        m_PacCoeff.C14 = d["Aligning Coefficients"]["c14"].GetDouble();
        m_PacCoeff.C15 = d["Aligning Coefficients"]["c15"].GetDouble();
        m_PacCoeff.C16 = d["Aligning Coefficients"]["c16"].GetDouble();
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

void Pac89Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_trimesh_shape->SetStatic(true);
        m_wheel->AddAsset(m_trimesh_shape);
    } else {
        ChPac89Tire::AddVisualizationAssets(vis);
    }
}

void Pac89Tire::RemoveVisualizationAssets() {
    ChPac89Tire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by Pac89Tire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_trimesh_shape);
    if (it != m_wheel->GetAssets().end())
        m_wheel->GetAssets().erase(it);
}
}  // namespace vehicle
}  // namespace chrono
