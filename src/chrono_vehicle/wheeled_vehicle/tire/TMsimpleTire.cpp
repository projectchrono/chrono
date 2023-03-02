// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
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
// TMsimpleTire tire constructed with data from file (JSON format).
//
// =============================================================================

#include <algorithm>

#include "chrono/core/ChLog.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMsimpleTire.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
TMsimpleTire::TMsimpleTire(const std::string& filename) : ChTMsimpleTire(""), m_has_mesh(false) {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

TMsimpleTire::TMsimpleTire(const rapidjson::Document& d) : ChTMsimpleTire(""), m_has_mesh(false) {
    Create(d);
}

void TMsimpleTire::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read design parameters (required)
    assert(d.HasMember("Design"));
    assert(d.HasMember("Coefficient of Friction"));
    assert(d.HasMember("Rolling Resistance Coefficient"));

    m_mass = d["Design"]["Mass [kg]"].GetDouble();
    m_inertia = ReadVectorJSON(d["Design"]["Inertia [kg.m2]"]);
    m_unloaded_radius = d["Design"]["Unloaded Radius [m]"].GetDouble();
    double rim_radius = d["Design"]["Rim Radius [m]"].GetDouble();
    m_width = d["Design"]["Width [m]"].GetDouble();

    double p_li = 1.0;
    double p_use = 1.0;

    if (d.HasMember("Parameters")) {
        // Full parameterization
        m_Fz_nom = d["Parameters"]["Tire Load"]["Nominal Vertical Force [N]"].GetDouble();
        m_Cz = d["Parameters"]["Tire Stiffness"]["Vertical [N/m]"].GetDouble();
        m_Dz = d["Parameters"]["Tire Damping"]["Vertical [Ns/m]"].GetDouble();
        m_dFx0_1 = d["Parameters"]["Longitudinal"]["Initial Slopes dFx/dsx [N]"][0u].GetDouble();
        m_dFx0_2 = d["Parameters"]["Longitudinal"]["Initial Slopes dFx/dsx [N]"][1u].GetDouble();
        m_Fx_max1 = d["Parameters"]["Longitudinal"]["Maximum Fx Load [N]"][0u].GetDouble();
        m_Fx_max2 = d["Parameters"]["Longitudinal"]["Maximum Fx Load [N]"][1u].GetDouble();
        m_Fx_inf1 = d["Parameters"]["Longitudinal"]["Sliding Fx Load [N]"][0u].GetDouble();
        m_Fx_inf2 = d["Parameters"]["Longitudinal"]["Sliding Fx Load [N]"][1u].GetDouble();
        m_dFy0_1 = d["Parameters"]["Lateral"]["Initial Slopes dFy/dsy [N]"][0u].GetDouble();
        m_dFy0_2 = d["Parameters"]["Lateral"]["Initial Slopes dFy/dsy [N]"][1u].GetDouble();
        m_Fy_max1 = d["Parameters"]["Lateral"]["Maximum Fy Load [N]"][0u].GetDouble();
        m_Fy_max2 = d["Parameters"]["Lateral"]["Maximum Fy Load [N]"][1u].GetDouble();
        m_Fy_inf1 = d["Parameters"]["Lateral"]["Sliding Fy Load [N]"][0u].GetDouble();
        m_Fy_inf2 = d["Parameters"]["Lateral"]["Sliding Fy Load [N]"][1u].GetDouble();
    } else if (d.HasMember("Load Index")) {
        // Information about tire inflation pressure might be present
        if (d.HasMember("Inflation Pressure Design [Pa]")) {
            p_li = d["Inflation Pressure Design [Pa]"].GetDouble();
        } else {
            p_li = 0.0;
        }
        if (d.HasMember("Inflation Pressure Use [Pa]")) {
            p_use = d["Inflation Pressure Use [Pa]"].GetDouble();
        } else {
            p_use = 0.0;
        }
        if (p_use > 0.0 && p_li > 0.0) {
            ////pressure_info_found = true;
        } else {
            p_li = p_use = 1.0;
        }
        // Specification through load index
        unsigned int li = d["Load Index"].GetUint();
        std::string vehicle_type = d["Vehicle Type"].GetString();
        if (vehicle_type.compare("Truck") == 0) {
            GuessTruck80Par(li, m_width, (m_unloaded_radius - rim_radius) / m_width, 2 * rim_radius, p_li, p_use);
        } else {
            GuessPassCar70Par(li, m_width, (m_unloaded_radius - rim_radius) / m_width, 2 * rim_radius, p_li, p_use);
        }
    } else if (d.HasMember("Maximum Bearing Capacity [N]")) {
        // Information about tire inflation pressure might be present
        if (d.HasMember("Inflation Pressure Design [Pa]")) {
            p_li = d["Inflation Pressure Design [Pa]"].GetDouble();
        } else {
            p_li = 0.0;
        }
        if (d.HasMember("Inflation Pressure Use [Pa]")) {
            p_use = d["Inflation Pressure Use [Pa]"].GetDouble();
        } else {
            p_use = 0.0;
        }
        if (p_use > 0.0 && p_li > 0.0) {
            ////pressure_info_found = true;
        } else {
            p_use = 1.0;
            p_li = 1.0;
        }
        // Specification through bearing capacity
        double bearing_capacity = d["Maximum Bearing Capacity [N]"].GetDouble();
        std::string vehicle_type = d["Name"].GetString();
        if (vehicle_type.compare("truck") == 0) {
            GuessTruck80Par(bearing_capacity, m_width, (m_unloaded_radius - rim_radius) / m_width, rim_radius, p_li,
                            p_use);
        } else {
            GuessPassCar70Par(bearing_capacity, m_width, (m_unloaded_radius - rim_radius) / m_width, rim_radius,
                              p_li, p_use);
        }
    } else {
        GetLog() << "ERROR: Incorrect TMeasy JSON specification.\n";
        return;
    }

    // Coefficient of friction and rolling resistance coefficients.
    // These must be set here to ensure they are not overwritten.
    m_mu_0 = d["Coefficient of Friction"].GetDouble();
    m_rolling_resistance = d["Rolling Resistance Coefficient"].GetDouble();

    // Check how to visualize this tire.
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh Filename Left") && d["Visualization"].HasMember("Mesh Filename Right")) {
            m_meshFile_left = d["Visualization"]["Mesh Filename Left"].GetString();
            m_meshFile_right = d["Visualization"]["Mesh Filename Right"].GetString();
            m_has_mesh = true;
        }
    }
    
    GenerateWorkParameters();
}

// -----------------------------------------------------------------------------
void TMsimpleTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMsimpleTire::AddVisualizationAssets(vis);
    }
}

void TMsimpleTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMsimpleTire::RemoveVisualizationAssets();
}

}  // end namespace vehicle
}  // end namespace chrono

