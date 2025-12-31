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
// BMW E90 TMeasy rear tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/bmw/BMW_E90_TMeasyTireRear.h"
#include "chrono_vehicle/ChVehicleDataPath.h"

namespace chrono {
namespace vehicle {
namespace bmw {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string BMW_E90_TMeasyTireRear::m_meshFile_left = "bmw/tire/bmw_e90_rear_Tire.obj";
const std::string BMW_E90_TMeasyTireRear::m_meshFile_right = "bmw/tire/bmw_e90_rear_Tire.obj";

const double BMW_E90_TMeasyTireRear::m_mass = 37.6;
const ChVector3d BMW_E90_TMeasyTireRear::m_inertia(3.84, 6.69, 3.84);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
BMW_E90_TMeasyTireRear::BMW_E90_TMeasyTireRear(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void BMW_E90_TMeasyTireRear::SetTMeasyParams() {
    m_par.pn = 3285.227750;
    m_par.pn_max = 11498.297125;
    m_unloaded_radius = 0.317850;
    m_width = 0.255000;
    m_rim_radius = 0.228600;
    m_bottom_radius = 0.233600;
    m_bottom_stiffness = 1073577.698426;
    SetVerticalStiffness(357859.232809);
    SetFrictionCoefficient(0.850000);
    SetRollingResistanceCoefficient(0.010000);
    m_par.dz = 191.334784;
    // longitudinal
    m_par.dfx0_pn = 114801.716933;
    m_par.dfx0_p2n = 212127.291559;
    m_par.fxm_pn = 4030.641700;
    m_par.fxm_p2n = 7869.338215;
    m_par.fxs_pn = 3118.106854;
    m_par.fxs_p2n = 5544.347735;
    m_par.sxm_pn = 0.112023;
    m_par.sxm_p2n = 0.111022;
    m_par.sxs_pn = 0.950000;
    m_par.sxs_p2n = 0.950000;
    // lateral
    m_par.dfy0_pn = 84164.224288;
    m_par.dfy0_p2n = 153728.545541;
    m_par.fym_pn = 3637.356228;
    m_par.fym_p2n = 6475.949639;
    m_par.fys_pn = 2585.674242;
    m_par.fys_p2n = 3810.706992;
    m_par.sym_pn = 0.126076;
    m_par.sym_p2n = 0.115674;
    m_par.sys_pn = 1.000000;
    m_par.sys_p2n = 1.000000;
    // alignment
    m_par.nL0_pn = 0.100826;
    m_par.nL0_p2n = 0.141748;
    m_par.sq0_pn = 0.133144;
    m_par.sq0_p2n = 0.152268;
    m_par.sqe_pn = 0.133144;
    m_par.sqe_p2n = 0.152268;
}

void BMW_E90_TMeasyTireRear::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/37x12.5x16.5_" + GetName() + ".gpl";
    WritePlots(filename, "37x12.5x16.5");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void BMW_E90_TMeasyTireRear::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void BMW_E90_TMeasyTireRear::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono
