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
// BMW E90 (330i 2006) TMsimple front tire subsystem 225/40 R18 92W
// Parameters calculated from NADS Tire Model Data Set
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/bmw/BMW_E90_TMsimpleTireFront.h"
#include "chrono_vehicle/ChVehicleDataPath.h"

namespace chrono {
namespace vehicle {
namespace bmw {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string BMW_E90_TMsimpleTireFront::m_meshFile_left = "bmw/tire/bmw_e90_front_Tire.obj";
const std::string BMW_E90_TMsimpleTireFront::m_meshFile_right = "bmw/tire/bmw_e90_front_Tire.obj";

const double BMW_E90_TMsimpleTireFront::m_mass = 11.2;
const ChVector3d BMW_E90_TMsimpleTireFront::m_inertia(0.495988, 0.897477, 0.495988);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
BMW_E90_TMsimpleTireFront::BMW_E90_TMsimpleTireFront(const std::string& name) : ChTMsimpleTire(name) {
    SetTMsimpleParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void BMW_E90_TMsimpleTireFront::SetTMsimpleParams() {
    double Cz = 310526.319544;  // linear tire stiffness N/m
    SetVerticalStiffness(Cz);

    m_unloaded_radius = 0.3186;  //$Free tyre radius
    m_width = 0.225000;          //$Nominal section width of the tyre
    m_rim_radius = 0.2286;       //$Nominal rim radius
    m_rolling_resistance = 0.01;
    m_bottom_radius = 0.240030;
    m_bottom_stiffness = 3.0 * Cz;

    m_par.dz = 186.491147;  // linear vertical damping coefficient Ns/m
    m_par.pn = 3089.094750;
    m_par.pn_max = 3.5 * m_par.pn;
    m_par.mu_0 = 0.85;

    m_par.dfx0_pn = 95453.800847;
    m_par.dfx0_p2n = 187961.001210;
    m_par.fxm_pn = 3804.204730;
    m_par.fxm_p2n = 7320.754100;
    m_par.fxs_pn = 2883.743455;
    m_par.fxs_p2n = 5026.523713;

    m_par.dfy0_pn = 68872.790069;
    m_par.dfy0_p2n = 125643.234662;
    m_par.fym_pn = 3432.500362;
    m_par.fym_p2n = 6112.008850;
    m_par.fys_pn = 2412.660879;
    m_par.fys_p2n = 3661.371397;

    SetHorizontalCoefficients();
}

void BMW_E90_TMsimpleTireFront::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "365_65_R20_60psi" + GetName() + ".gpl";
    WritePlots(filename, "365_65_R20_60psi");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void BMW_E90_TMsimpleTireFront::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMsimpleTire::AddVisualizationAssets(vis);
    }
}

void BMW_E90_TMsimpleTireFront::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMsimpleTire::RemoveVisualizationAssets();
}

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono
