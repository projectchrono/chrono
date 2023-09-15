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
// TMeasyTire tire constructed with data from file (JSON format).
//
// =============================================================================

#include <algorithm>

#include "chrono/core/ChLog.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
TMeasyTire::TMeasyTire(const std::string& filename) : ChTMeasyTire(""), m_has_mesh(false) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

TMeasyTire::TMeasyTire(const rapidjson::Document& d) : ChTMeasyTire(""), m_has_mesh(false) {
    Create(d);
}

void TMeasyTire::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read design parameters (required)
    assert(d.HasMember("Design"));

    m_mass = d["Design"]["Mass [kg]"].GetDouble();
    m_inertia = ReadVectorJSON(d["Design"]["Inertia [kg.m2]"]);
    m_unloaded_radius = d["Design"]["Unloaded Radius [m]"].GetDouble();
    m_rim_radius = d["Design"]["Rim Radius [m]"].GetDouble();
    m_width = d["Design"]["Width [m]"].GetDouble();

    double p_li = 1.0;
    double p_use = 1.0;

    if (d.HasMember("Parameters")) {
        // Full parameterization
        m_par.pn = d["Parameters"]["Vertical"]["Nominal Vertical Force [N]"].GetDouble();
        if (d["Parameters"]["Vertical"].HasMember("Maximum Vertical Force [N]"))
            m_par.pn_max = d["Parameters"]["Vertical"]["Maximum Vertical Force [N]"].GetDouble();
        else
            m_par.pn_max = 3.5 * m_par.pn;

        if (d["Parameters"]["Vertical"].HasMember("Vertical Tire Stiffness [N/m]")) {
            double a1 = d["Parameters"]["Vertical"]["Vertical Tire Stiffness [N/m]"].GetDouble();
            SetVerticalStiffness(a1);
        } else if (d["Parameters"]["Vertical"].HasMember("Tire Spring Curve Data")) {
            assert(d["Parameters"]["Vertical"]["Tire Spring Curve Data"].IsArray() &&
                   d["Parameters"]["Vertical"]["Tire Spring Curve Data"][0u].Size() == 2);
            std::vector<double> defl, force;
            int num_defs = d["Parameters"]["Vertical"]["Tire Spring Curve Data"].Size();
            for (int i = 0; i < num_defs; i++) {
                double t_defl = d["Parameters"]["Vertical"]["Tire Spring Curve Data"][i][0u].GetDouble();
                double t_force = d["Parameters"]["Vertical"]["Tire Spring Curve Data"][i][1u].GetDouble();
                defl.push_back(t_defl);
                force.push_back(t_force);
            }
            SetVerticalStiffness(defl, force);
            GetLog() << "Table set\n";
        } else {
            GetLog() << "FATAL: No Vertical Tire Stiffness Definition!\n";
            exit(19);
        }

        m_par.dz = d["Parameters"]["Vertical"]["Tire Vertical Damping [Ns/m]"].GetDouble();

        // Bottoming parameters are optional, if not set, Initialize() sets default values
        if (d["Parameters"]["Vertical"].HasMember("Tire Bottoming Radius [m]"))
            m_bottom_radius = d["Parameters"]["Vertical"]["Tire Bottoming Radius [m]"].GetDouble();
        if (d["Parameters"]["Vertical"].HasMember("Tire Bottoming Stiffness [N/m]"))
            m_bottom_stiffness = d["Parameters"]["Vertical"]["Tire Bottoming Stiffness [N/m]"].GetDouble();

        m_par.dfx0_pn = d["Parameters"]["Longitudinal"]["Initial Slopes dFx/dsx [N]"][0u].GetDouble();
        m_par.dfx0_p2n = d["Parameters"]["Longitudinal"]["Initial Slopes dFx/dsx [N]"][1u].GetDouble();
        m_par.fxm_pn = d["Parameters"]["Longitudinal"]["Maximum Fx Load [N]"][0u].GetDouble();
        m_par.fxm_p2n = d["Parameters"]["Longitudinal"]["Maximum Fx Load [N]"][1u].GetDouble();
        m_par.fxs_pn = d["Parameters"]["Longitudinal"]["Sliding Fx Load [N]"][0u].GetDouble();
        m_par.fxs_p2n = d["Parameters"]["Longitudinal"]["Sliding Fx Load [N]"][1u].GetDouble();
        m_par.sxm_pn = d["Parameters"]["Longitudinal"]["Slip sx at Maximum Fx"][0u].GetDouble();
        m_par.sxm_p2n = d["Parameters"]["Longitudinal"]["Slip sx at Maximum Fx"][1u].GetDouble();
        m_par.sxs_pn = d["Parameters"]["Longitudinal"]["Slip sx where sliding begins"][0u].GetDouble();
        m_par.sxs_p2n = d["Parameters"]["Longitudinal"]["Slip sx where sliding begins"][1u].GetDouble();

        m_par.dfy0_pn = d["Parameters"]["Lateral"]["Initial Slopes dFy/dsy [N]"][0u].GetDouble();
        m_par.dfy0_p2n = d["Parameters"]["Lateral"]["Initial Slopes dFy/dsy [N]"][1u].GetDouble();
        m_par.fym_pn = d["Parameters"]["Lateral"]["Maximum Fy Load [N]"][0u].GetDouble();
        m_par.fym_p2n = d["Parameters"]["Lateral"]["Maximum Fy Load [N]"][1u].GetDouble();
        m_par.fys_pn = d["Parameters"]["Lateral"]["Sliding Fy Load [N]"][0u].GetDouble();
        m_par.fys_p2n = d["Parameters"]["Lateral"]["Sliding Fy Load [N]"][1u].GetDouble();
        m_par.sym_pn = d["Parameters"]["Lateral"]["Slip sy at Maximum Fy"][0u].GetDouble();
        m_par.sym_p2n = d["Parameters"]["Lateral"]["Slip sy at Maximum Fy"][1u].GetDouble();
        m_par.sys_pn = d["Parameters"]["Lateral"]["Slip sy where sliding begins"][0u].GetDouble();
        m_par.sys_p2n = d["Parameters"]["Lateral"]["Slip sy where sliding begins"][1u].GetDouble();

        m_par.nL0_pn = d["Parameters"]["Aligning"]["Normalized Trail at Zero Slip sy"][0u].GetDouble();
        m_par.nL0_p2n = d["Parameters"]["Aligning"]["Normalized Trail at Zero Slip sy"][1u].GetDouble();
        m_par.sq0_pn = d["Parameters"]["Aligning"]["Slip sy where Trail Changes Sign"][0u].GetDouble();
        m_par.sq0_p2n = d["Parameters"]["Aligning"]["Slip sy where Trail Changes Sign"][1u].GetDouble();
        m_par.sqe_pn = d["Parameters"]["Aligning"]["Slip sy where Trail Tends to Zero"][0u].GetDouble();
        m_par.sqe_p2n = d["Parameters"]["Aligning"]["Slip sy where Trail Tends to Zero"][1u].GetDouble();

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
    if (d.HasMember("Coefficient of Friction"))
        m_par.mu_0 = d["Coefficient of Friction"].GetDouble();
    if (d.HasMember("Rolling Resistance Coefficient"))
        m_rolling_resistance = d["Rolling Resistance Coefficient"].GetDouble();

    m_visualization_width = ChTMeasyTire::GetVisualizationWidth();

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

// -----------------------------------------------------------------------------
void TMeasyTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void TMeasyTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

}  // end namespace vehicle
}  // end namespace chrono
