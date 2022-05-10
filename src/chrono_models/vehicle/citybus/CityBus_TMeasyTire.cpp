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
// Authors: Rainer Gericke, Asher Elmquist, Evan Hoerl, Shuo He
// =============================================================================
//
// CityBus TMeasy tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/citybus/CityBus_TMeasyTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace citybus {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string CityBus_TMeasyTire::m_meshFile = "citybus/CityBusTire.obj";

const double CityBus_TMeasyTire::m_mass = 68.6;
const ChVector<> CityBus_TMeasyTire::m_inertia(6.104, 11.144, 6.104);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
CityBus_TMeasyTire::CityBus_TMeasyTire(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void CityBus_TMeasyTire::SetTMeasyParams() {
    ////unsigned int li = 152;
    const double in2m = 0.0254;
    double h = (1.05 - 16 * in2m) / 2.0;
    double w = 0.295;
    double r = h / w;
    double rimdia = 16.0 * in2m;
    double pinfl_li = 850.0 * 1000;
    double pinfl_use = 850.0 * 4000;

    double load = 45000;

    GuessTruck80Par(load,    // tire load [N]
                    w,       // tire width [m]
                    r,       // aspect ratio []
                    rimdia,  // rim diameter [m]
                    pinfl_li, pinfl_use);
}

void CityBus_TMeasyTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/37x12.5x16.5_" + GetName() + ".gpl";
    WritePlots(filename, "37x12.5x16.5");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void CityBus_TMeasyTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void CityBus_TMeasyTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
