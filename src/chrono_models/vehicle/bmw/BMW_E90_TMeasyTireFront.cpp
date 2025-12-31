//
// Created by Rainer Gericke on 04.06.24.
//

#include "BMW_E90_TMeasyTireFront.h"
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
// BMW E90 TMeasy tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/bmw/BMW_E90_TMeasyTireFront.h"
#include "chrono_vehicle/ChVehicleDataPath.h"

namespace chrono {
namespace vehicle {
namespace bmw {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string BMW_E90_TMeasyTireFront::m_meshFile_left = "bmw/tire/bmw_e90_front_Tire.obj";
const std::string BMW_E90_TMeasyTireFront::m_meshFile_right = "bmw/tire/bmw_e90_front_Tire.obj";

const double BMW_E90_TMeasyTireFront::m_mass = 37.6;
const ChVector3d BMW_E90_TMeasyTireFront::m_inertia(3.84, 6.69, 3.84);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
BMW_E90_TMeasyTireFront::BMW_E90_TMeasyTireFront(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void BMW_E90_TMeasyTireFront::SetTMeasyParams() {
    m_par.pn = 3089.094750;
    m_par.pn_max = 10811.831625;
    m_unloaded_radius = 0.318600;
    m_width = 0.225000;
    m_rim_radius = 0.228600;
    m_bottom_radius = 0.233600;
    m_bottom_stiffness = 931578.958633;
    SetVerticalStiffness(310526.319544);
    SetFrictionCoefficient(0.850000);
    SetRollingResistanceCoefficient(0.010000);
    m_par.dz = 186.491147;
    // longitudinal
    m_par.dfx0_pn = 115219.880802;
    m_par.dfx0_p2n = 223456.348293;
    m_par.fxm_pn = 3804.775171;
    m_par.fxm_p2n = 7315.557946;
    m_par.fxs_pn = 2883.743455;
    m_par.fxs_p2n = 5026.523713;
    m_par.sxm_pn = 0.106017;
    m_par.sxm_p2n = 0.100500;
    m_par.sxs_pn = 0.950000;
    m_par.sxs_p2n = 0.950000;
    // lateral
    m_par.dfy0_pn = 87137.208230;
    m_par.dfy0_p2n = 157051.536207;
    m_par.fym_pn = 3423.092559;
    m_par.fym_p2n = 6094.402570;
    m_par.fys_pn = 2412.660879;
    m_par.fys_p2n = 3661.371397;
    m_par.sym_pn = 0.126939;
    m_par.sym_p2n = 0.119538;
    m_par.sys_pn = 1.000000;
    m_par.sys_p2n = 1.000000;
    // alignment
    m_par.nL0_pn = 0.106964;
    m_par.nL0_p2n = 0.153714;
    m_par.sq0_pn = 0.134683;
    m_par.sq0_p2n = 0.155617;
    m_par.sqe_pn = 0.134683;
    m_par.sqe_p2n = 0.155617;
}

void BMW_E90_TMeasyTireFront::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/37x12.5x16.5_" + GetName() + ".gpl";
    WritePlots(filename, "37x12.5x16.5");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void BMW_E90_TMeasyTireFront::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void BMW_E90_TMeasyTireFront::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono
