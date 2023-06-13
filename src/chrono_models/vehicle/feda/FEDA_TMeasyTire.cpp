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
// Authors: Rainer Gericke
// =============================================================================
//
// FEDA TMeasy tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/feda/FEDA_TMeasyTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string FEDA_TMeasyTire::m_meshFile_left = "feda/meshes/FEDA_tire_fine.obj";
const std::string FEDA_TMeasyTire::m_meshFile_right = "feda/meshes/FEDA_tire_fine.obj";

const double FEDA_TMeasyTire::m_mass = 56.1;
const ChVector<> FEDA_TMeasyTire::m_inertia(10.5, 16.8, 10.5);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FEDA_TMeasyTire::FEDA_TMeasyTire(const std::string& name) : ChTMeasyTire(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void FEDA_TMeasyTire::SetTMeasyParams() {
    std::vector<double> defl = {0,     0.005, 0.01,  0.015, 0.02,  0.025, 0.03,  0.035, 0.04,  0.045, 0.05,
                                0.055, 0.06,  0.065, 0.07,  0.075, 0.08,  0.085, 0.09,  0.095, 0.1};
    std::vector<double> frc = {0.00,     2004.06,  4242.26,  6688.46,  9316.51,  12100.25, 15013.54,
                               18030.23, 21124.16, 24269.19, 27439.17, 30607.94, 33749.36, 36837.28,
                               39845.54, 42748.00, 45518.51, 48130.91, 50559.06, 52776.81, 54758.00};

    // TMeasy settings
    m_unloaded_radius = 0.4987;
    m_width = 0.335;
    m_rim_radius = 0.2858;
    m_rolling_resistance = 0.015;
    m_par.mu_0 = 0.8;
    m_par.pn = 10837;
    m_par.pn_max = 37929.5;
    SetVerticalStiffness(defl,frc);
    m_par.dz = 6188;
    m_par.dfx0_pn = 89324.1244;
    m_par.dfx0_p2n = 170911.0652;
    m_par.fxm_pn = 10356.6344;
    m_par.fxm_p2n = 20238.7957;
    m_par.fxs_pn = 8598.812;
    m_par.fxs_p2n = 16604.7647;
    m_par.sxm_pn = 0.16807;
    m_par.sxm_p2n = 0.13913;
    m_par.sxs_pn = 0.66667;
    m_par.sxs_p2n = 0.66667;
    m_par.dfy0_pn = 141666.3303;
    m_par.dfy0_p2n = 257131.1695;
    m_par.fym_pn = 8351.7213;
    m_par.fym_p2n = 15624.4596;
    m_par.fys_pn = 8073.6063;
    m_par.fys_p2n = 15157.9197;
    m_par.sym_pn = 0.22834;
    m_par.sym_p2n = 0.23905;
    m_par.sys_pn = 1.0296;
    m_par.sys_p2n = 1.0296;
    m_par.nL0_pn = 0.178;
    m_par.sq0_pn = 0.25117;
    m_par.sqe_pn = 1.0296;
    m_par.nL0_p2n = 0.19;
    m_par.sq0_p2n = 0.26296;
    m_par.sqe_p2n = 1.0296;
}

void FEDA_TMeasyTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "365_65_R20_60psi" + GetName() + ".gpl";
    WritePlots(filename, "365_65_R20_60psi");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void FEDA_TMeasyTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void FEDA_TMeasyTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono
