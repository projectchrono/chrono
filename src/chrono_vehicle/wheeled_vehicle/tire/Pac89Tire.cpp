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

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Pac89Tire::Pac89Tire(const std::string& filename) : ChPac89Tire(""), m_mass(0), m_normalDamping(0), m_has_mesh(false) {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

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
        m_PacCoeff.A13 = d["Lateral Coefficients"]["a13"].GetDouble();
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
        m_PacCoeff.C17 = d["Aligning Coefficients"]["c17"].GetDouble();
    }

    m_visualization_width = ChPac89Tire::GetVisualizationWidth();

    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh Filename Left") && d["Visualization"].HasMember("Mesh Filename Right")) {
            m_meshFile_left = d["Visualization"]["Mesh Filename Left"].GetString();
            m_meshFile_right = d["Visualization"]["Mesh Filename Right"].GetString();
            m_has_mesh = true;
        }

        if (d["Visualization"].HasMember("Width")) {
            m_visualization_width = d["Visualization"]["Width"].GetDouble();
        }
    }
}

void Pac89Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChPac89Tire::AddVisualizationAssets(vis);
    }
}

void Pac89Tire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChPac89Tire::RemoveVisualizationAssets();
}

}  // namespace vehicle
}  // namespace chrono
