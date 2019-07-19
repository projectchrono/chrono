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

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
TMeasyTire::TMeasyTire(const std::string& filename) : ChTMeasyTire(""), m_has_mesh(false) {
    FILE* fp = fopen(filename.c_str(), "r");
    if (fp == nullptr) {
        GetLog() << "TMeasy Data File not found!\n";
    }

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

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
    assert(d.HasMember("Coefficient of Friction"));
    assert(d.HasMember("Rolling Resistance Coefficients"));

    m_mass = d["Design"]["Mass [kg]"].GetDouble();
    m_inertia = ReadVectorJSON(d["Design"]["Inertia [kg.m2]"]);
    m_unloaded_radius = d["Design"]["Unloaded Radius [m]"].GetDouble();
    m_rim_radius = d["Design"]["Rim Radius [m]"].GetDouble();
    m_width = d["Design"]["Width [m]"].GetDouble();
    m_roundness = d["Design"]["Roundness of Cross Section"].GetDouble();

    double p_li = 1.0;
    double p_use = 1.0;
    bool pressure_info_found = false;

    if (d.HasMember("Parameters")) {
        // Full parameterization
        const double N2kN = 0.001;

        m_TMeasyCoeff.pn = d["Parameters"]["Tire Load"]["Nominal Vertical Force [N]"].GetDouble();
        m_TMeasyCoeff.pn_max = d["Parameters"]["Tire Load"]["Maximum Vertical Force [N]"].GetDouble();

        m_TMeasyCoeff.cx = d["Parameters"]["Tire Stiffness"]["Longitudinal [N/m]"].GetDouble();
        m_TMeasyCoeff.cy = d["Parameters"]["Tire Stiffness"]["Lateral [N/m]"].GetDouble();
        double a1 = d["Parameters"]["Tire Stiffness"]["Vertical [N/m]"][0u].GetDouble();
        double a2 = d["Parameters"]["Tire Stiffness"]["Vertical [N/m]"][1u].GetDouble();
        SetVerticalStiffness(a1, a2);

        m_TMeasyCoeff.dx = d["Parameters"]["Tire Damping"]["Longitudinal [Ns/m]"].GetDouble();
        m_TMeasyCoeff.dy = d["Parameters"]["Tire Damping"]["Lateral [Ns/m]"].GetDouble();
        m_TMeasyCoeff.dz = d["Parameters"]["Tire Damping"]["Vertical [Ns/m]"].GetDouble();

        m_TMeasyCoeff.rdynco_pn = d["Parameters"]["Dynamic Radius Weighting Coefficients"][0u].GetDouble();
        m_TMeasyCoeff.rdynco_p2n = d["Parameters"]["Dynamic Radius Weighting Coefficients"][1u].GetDouble();

        m_TMeasyCoeff.dfx0_pn = N2kN * d["Parameters"]["Longitudinal"]["Initial Slopes dFx/dsx [N]"][0u].GetDouble();
        m_TMeasyCoeff.dfx0_p2n = N2kN * d["Parameters"]["Longitudinal"]["Initial Slopes dFx/dsx [N]"][1u].GetDouble();
        m_TMeasyCoeff.fxm_pn = N2kN * d["Parameters"]["Longitudinal"]["Maximum Fx Load [N]"][0u].GetDouble();
        m_TMeasyCoeff.fxm_p2n = N2kN * d["Parameters"]["Longitudinal"]["Maximum Fx Load [N]"][1u].GetDouble();
        m_TMeasyCoeff.fxs_pn = N2kN * d["Parameters"]["Longitudinal"]["Sliding Fx Load [N]"][0u].GetDouble();
        m_TMeasyCoeff.fxs_p2n = N2kN * d["Parameters"]["Longitudinal"]["Sliding Fx Load [N]"][1u].GetDouble();
        m_TMeasyCoeff.sxm_pn = d["Parameters"]["Longitudinal"]["Slip sx at Maximum Fx"][0u].GetDouble();
        m_TMeasyCoeff.sxm_p2n = d["Parameters"]["Longitudinal"]["Slip sx at Maximum Fx"][1u].GetDouble();
        m_TMeasyCoeff.sxs_pn = d["Parameters"]["Longitudinal"]["Slip sx where sliding begins"][0u].GetDouble();
        m_TMeasyCoeff.sxs_p2n = d["Parameters"]["Longitudinal"]["Slip sx where sliding begins"][1u].GetDouble();

        m_TMeasyCoeff.dfy0_pn = N2kN * d["Parameters"]["Lateral"]["Initial Slopes dFy/dsy [N]"][0u].GetDouble();
        m_TMeasyCoeff.dfy0_p2n = N2kN * d["Parameters"]["Lateral"]["Initial Slopes dFy/dsy [N]"][1u].GetDouble();
        m_TMeasyCoeff.fym_pn = N2kN * d["Parameters"]["Lateral"]["Maximum Fy Load [N]"][0u].GetDouble();
        m_TMeasyCoeff.fym_p2n = N2kN * d["Parameters"]["Lateral"]["Maximum Fy Load [N]"][1u].GetDouble();
        m_TMeasyCoeff.fys_pn = N2kN * d["Parameters"]["Lateral"]["Sliding Fy Load [N]"][0u].GetDouble();
        m_TMeasyCoeff.fys_p2n = N2kN * d["Parameters"]["Lateral"]["Sliding Fy Load [N]"][1u].GetDouble();
        m_TMeasyCoeff.sym_pn = d["Parameters"]["Lateral"]["Slip sy at Maximum Fy"][0u].GetDouble();
        m_TMeasyCoeff.sym_p2n = d["Parameters"]["Lateral"]["Slip sy at Maximum Fy"][1u].GetDouble();
        m_TMeasyCoeff.sys_pn = d["Parameters"]["Lateral"]["Slip sy where sliding begins"][0u].GetDouble();
        m_TMeasyCoeff.sys_p2n = d["Parameters"]["Lateral"]["Slip sy where sliding begins"][1u].GetDouble();

        m_TMeasyCoeff.nto0_pn = d["Parameters"]["Aligning"]["Normalized Trail at Zero Slip sy"][0u].GetDouble();
        m_TMeasyCoeff.nto0_p2n = d["Parameters"]["Aligning"]["Normalized Trail at Zero Slip sy"][1u].GetDouble();
        m_TMeasyCoeff.synto0_pn = d["Parameters"]["Aligning"]["Slip sy where Trail Changes Sign"][0u].GetDouble();
        m_TMeasyCoeff.synto0_p2n = d["Parameters"]["Aligning"]["Slip sy where Trail Changes Sign"][1u].GetDouble();
        m_TMeasyCoeff.syntoE_pn = d["Parameters"]["Aligning"]["Slip sy where Trail Tends to Zero"][0u].GetDouble();
        m_TMeasyCoeff.syntoE_p2n = d["Parameters"]["Aligning"]["Slip sy where Trail Tends to Zero"][1u].GetDouble();

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
            pressure_info_found = true;
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
            pressure_info_found = true;
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
        std::cout << "ERROR: Incorrect TMeasy JSON specification." << std::endl;
        return;
    }

    // Coefficient of friction and rolling resistance coefficients.
    // These must be set here to ensure they are not overwritten.
    m_TMeasyCoeff.mu_0 = d["Coefficient of Friction"].GetDouble();
    m_TMeasyCoeff.rrcoeff_pn = d["Rolling Resistance Coefficients"][0u].GetDouble();
    m_TMeasyCoeff.rrcoeff_p2n = d["Rolling Resistance Coefficients"][1u].GetDouble();

    // Check how to visualize this tire.
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh Filename")) {
            m_meshFile = d["Visualization"]["Mesh Filename"].GetString();
            m_meshName = d["Visualization"]["Mesh Name"].GetString();
            m_has_mesh = true;
        }
    }
}

// -----------------------------------------------------------------------------
void TMeasyTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_trimesh_shape->SetStatic(true);
        m_wheel->AddAsset(m_trimesh_shape);
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void TMeasyTire::RemoveVisualizationAssets() {
    ChTMeasyTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by RigidTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_trimesh_shape);
    if (it != m_wheel->GetAssets().end())
        m_wheel->GetAssets().erase(it);
}

}  // end namespace vehicle
}  // end namespace chrono
