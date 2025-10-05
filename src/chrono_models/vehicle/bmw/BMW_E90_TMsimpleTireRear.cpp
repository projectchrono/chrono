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
// BMW E90 (330i 2006) TMsimple rear tire subsystem 255/35 R18 94W
// Parameters calculated from NADS Tire Model Data Set
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/bmw/BMW_E90_TMsimpleTireRear.h"
#include "chrono_vehicle/ChVehicleDataPath.h"

namespace chrono {
namespace vehicle {
namespace bmw {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string BMW_E90_TMsimpleTireRear::m_meshFile_left = "bmw/tire/bmw_e90_rear_Tire.obj";
const std::string BMW_E90_TMsimpleTireRear::m_meshFile_right = "bmw/tire/bmw_e90_rear_Tire.obj";

const double BMW_E90_TMsimpleTireRear::m_mass = 10.23;
const ChVector3d BMW_E90_TMsimpleTireRear::m_inertia(0.463159, 0.815450, 0.463159);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
BMW_E90_TMsimpleTireRear::BMW_E90_TMsimpleTireRear(const std::string& name) : ChTMsimpleTire(name) {
    SetTMsimpleParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void BMW_E90_TMsimpleTireRear::SetTMsimpleParams() {
    double Cz = 357859.232809;  // linear tire stiffness N/m
    SetVerticalStiffness(Cz);

    m_unloaded_radius = 0.3186;  //$Free tyre radius
    m_width = 0.255000;          //$Nominal section width of the tyre
    m_rim_radius = 0.2286;       //$Nominal rim radius
    m_rolling_resistance = 0.01;
    m_bottom_radius = 0.240030;
    m_bottom_stiffness = 3.0 * Cz;

    m_par.dz = 191.334784;  // linear vertical damping coefficient Ns/m
    m_par.pn = 3285.227750;
    m_par.pn_max = 3.5 * m_par.pn;
    m_par.mu_0 = 0.85;

    m_par.dfx0_pn = 95426.247934;
    m_par.dfx0_p2n = 180428.660916;
    m_par.fxm_pn = 4030.192524;
    m_par.fxm_p2n = 7875.872679;
    m_par.fxs_pn = 3118.106854;
    m_par.fxs_p2n = 5544.347735;

    m_par.dfy0_pn = 73011.000908;
    m_par.dfy0_p2n = 135869.397227;
    m_par.fym_pn = 3647.834933;
    m_par.fym_p2n = 6498.518198;
    m_par.fys_pn = 2585.674242;
    m_par.fys_p2n = 3810.706992;

    SetHorizontalCoefficients();
}

void BMW_E90_TMsimpleTireRear::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "365_65_R20_60psi" + GetName() + ".gpl";
    WritePlots(filename, "365_65_R20_60psi");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void BMW_E90_TMsimpleTireRear::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMsimpleTire::AddVisualizationAssets(vis);
    }
}

void BMW_E90_TMsimpleTireRear::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMsimpleTire::RemoveVisualizationAssets();
}

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono
