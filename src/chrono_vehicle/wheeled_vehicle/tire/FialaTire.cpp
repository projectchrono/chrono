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

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FialaTire::FialaTire(const std::string& filename) : ChFialaTire(""), m_has_vert_table(false), m_has_mesh(false) {
    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

FialaTire::FialaTire(const rapidjson::Document& d) : ChFialaTire(""), m_has_vert_table(false), m_has_mesh(false) {
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
    if (d["Fiala Parameters"].HasMember("Vertical Curve Data")) {
        int num_points = d["Fiala Parameters"]["Vertical Curve Data"].Size();
        auto data = d["Fiala Parameters"]["Vertical Curve Data"].GetArray();
        for (int i = 0; i < num_points; i++) {
            m_vert_map.AddPoint(data[i][0u].GetDouble(), data[i][1u].GetDouble());
        }
        auto pnts = m_vert_map.GetPoints();
        m_max_depth = data[num_points - 1][0u].GetDouble();
        m_max_val = data[num_points - 1][1u].GetDouble();
        m_slope = (data[num_points - 1][1u].GetDouble() - data[num_points - 2][1u].GetDouble()) /
                  (data[num_points - 1][0u].GetDouble() - data[num_points - 2][0u].GetDouble());
        m_has_vert_table = true;
    }
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

    // Check how to visualize this tire.
    m_visualization_width = m_width;

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

double FialaTire::GetNormalStiffnessForce(double depth) const {
    if (m_has_vert_table) {
        if (depth > m_max_depth) {
            // Linear extrapolation beyond available depth data
            return m_max_val + m_slope * (depth - m_max_depth);
        } else {
            // Return interpolated data
            return m_vert_map.Get_y(depth);
        }
    }

    // Linear model
    return m_normalStiffness * depth;
}

double FialaTire::GetNormalDampingForce(double depth, double velocity) const {
    return m_normalDamping * velocity;
}

// -----------------------------------------------------------------------------
void FialaTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        trimesh->Transform(ChVector<>(0, GetOffset(), 0), ChMatrix33<>(1));
        m_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_trimesh_shape->SetStatic(true);
        m_wheel->GetSpindle()->AddAsset(m_trimesh_shape);
    } else {
        ChFialaTire::AddVisualizationAssets(vis);
    }
}

void FialaTire::RemoveVisualizationAssets() {
    ChFialaTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by FialaTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto& assets = m_wheel->GetSpindle()->GetAssets();
    auto it = std::find(assets.begin(), assets.end(), m_trimesh_shape);
    if (it != assets.end())
        assets.erase(it);
}

}  // end namespace vehicle
}  // end namespace chrono
