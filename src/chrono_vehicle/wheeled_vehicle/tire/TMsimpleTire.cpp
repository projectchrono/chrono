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
    Document d;
    ReadFileJSON(filename, d);
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
    m_rim_radius = d["Design"]["Rim Radius [m]"].GetDouble();
    m_width = d["Design"]["Width [m]"].GetDouble();

    double p_li = 1.0;
    double p_use = 1.0;
    ////bool pressure_info_found = false;

    if (d.HasMember("Parameters")) {
        // Full parameterization
        m_TMsimpleCoeff.pn = d["Parameters"]["Tire Load"]["Nominal Vertical Force [N]"].GetDouble();
        m_TMsimpleCoeff.pn_max = d["Parameters"]["Tire Load"]["Maximum Vertical Force [N]"].GetDouble();

        m_TMsimpleCoeff.cx = d["Parameters"]["Tire Stiffness"]["Longitudinal [N/m]"].GetDouble();
        m_TMsimpleCoeff.cy = d["Parameters"]["Tire Stiffness"]["Lateral [N/m]"].GetDouble();
        double a1 = d["Parameters"]["Tire Stiffness"]["Vertical [N/m]"][0u].GetDouble();
        double a2 = d["Parameters"]["Tire Stiffness"]["Vertical [N/m]"][1u].GetDouble();
        SetVerticalStiffness(a1, a2);

        m_TMsimpleCoeff.dx = d["Parameters"]["Tire Damping"]["Longitudinal [Ns/m]"].GetDouble();
        m_TMsimpleCoeff.dy = d["Parameters"]["Tire Damping"]["Lateral [Ns/m]"].GetDouble();
        m_TMsimpleCoeff.dz = d["Parameters"]["Tire Damping"]["Vertical [Ns/m]"].GetDouble();

        m_TMsimpleCoeff.dfx0_pn = d["Parameters"]["Longitudinal"]["Initial Slopes dFx/dsx [N]"][0u].GetDouble();
        m_TMsimpleCoeff.dfx0_p2n = d["Parameters"]["Longitudinal"]["Initial Slopes dFx/dsx [N]"][1u].GetDouble();
        m_TMsimpleCoeff.fxm_pn = d["Parameters"]["Longitudinal"]["Maximum Fx Load [N]"][0u].GetDouble();
        m_TMsimpleCoeff.fxm_p2n = d["Parameters"]["Longitudinal"]["Maximum Fx Load [N]"][1u].GetDouble();
        m_TMsimpleCoeff.fxs_pn = d["Parameters"]["Longitudinal"]["Sliding Fx Load [N]"][0u].GetDouble();
        m_TMsimpleCoeff.fxs_p2n = d["Parameters"]["Longitudinal"]["Sliding Fx Load [N]"][1u].GetDouble();

        m_TMsimpleCoeff.dfy0_pn = d["Parameters"]["Lateral"]["Initial Slopes dFy/dsy [N]"][0u].GetDouble();
        m_TMsimpleCoeff.dfy0_p2n = d["Parameters"]["Lateral"]["Initial Slopes dFy/dsy [N]"][1u].GetDouble();
        m_TMsimpleCoeff.fym_pn = d["Parameters"]["Lateral"]["Maximum Fy Load [N]"][0u].GetDouble();
        m_TMsimpleCoeff.fym_p2n = d["Parameters"]["Lateral"]["Maximum Fy Load [N]"][1u].GetDouble();
        m_TMsimpleCoeff.fys_pn = d["Parameters"]["Lateral"]["Sliding Fy Load [N]"][0u].GetDouble();
        m_TMsimpleCoeff.fys_p2n = d["Parameters"]["Lateral"]["Sliding Fy Load [N]"][1u].GetDouble();
        SetHorizontalCoefficients();
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
            GuessTruck80Par(li, m_width, (m_unloaded_radius - m_rim_radius) / m_width, 2 * m_rim_radius, p_li, p_use);
        } else {
            GuessPassCar70Par(li, m_width, (m_unloaded_radius - m_rim_radius) / m_width, 2 * m_rim_radius, p_li, p_use);
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
            GuessTruck80Par(bearing_capacity, m_width, (m_unloaded_radius - m_rim_radius) / m_width, m_rim_radius, p_li,
                            p_use);
        } else {
            GuessPassCar70Par(bearing_capacity, m_width, (m_unloaded_radius - m_rim_radius) / m_width, m_rim_radius,
                              p_li, p_use);
        }
    } else {
        GetLog() << "ERROR: Incorrect TMeasy JSON specification.\n";
        return;
    }

    // Coefficient of friction and rolling resistance coefficients.
    // These must be set here to ensure they are not overwritten.
    m_TMsimpleCoeff.mu_0 = d["Coefficient of Friction"].GetDouble();
    m_rolling_resistance = d["Rolling Resistance Coefficient"].GetDouble();

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
