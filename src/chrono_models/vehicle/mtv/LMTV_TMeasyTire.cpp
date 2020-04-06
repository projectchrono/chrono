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
// LMTV TMeasy tire subsystem 395/85R20 159G 655 kPa
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/mtv/LMTV_TMeasyTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace mtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string LMTV_TMeasyTire::m_meshFile = "mtv/meshes/MTV_tire.obj";

const double LMTV_TMeasyTire::m_mass = 110.0;
const ChVector<> LMTV_TMeasyTire::m_inertia(16.9, 29.5, 16.9);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
LMTV_TMeasyTire::LMTV_TMeasyTire(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void LMTV_TMeasyTire::SetTMeasyParams() {
    const double lbs2N = 4.4482216153;
    unsigned int li = 159;
    const double in2m = 0.0254;
    double w = 0.395;
    double r = 0.85;
    double rimdia = 20.0 * in2m;
    double pinfl_li = 655.0 * 1000;
    double pinfl_use = 655.0 * 1000;

    GuessTruck80Par(li,      // tire load index
                    w,       // tire width [m]
                    r,       // aspect ratio []
                    rimdia,  // rim diameter [m]
                    pinfl_li, pinfl_use);
}

void LMTV_TMeasyTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/395_85R20_" + GetName() + ".gpl";
    WritePlots(filename, "395/85R20");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void LMTV_TMeasyTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(vehicle::GetDataFile(m_meshFile),   // left side
                                               vehicle::GetDataFile(m_meshFile));  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void LMTV_TMeasyTire::RemoveVisualizationAssets() {
    ChTMeasyTire::RemoveVisualizationAssets();
    RemoveVisualizationMesh(m_trimesh_shape);
}

}  // namespace mtv
}  // end namespace vehicle
}  // end namespace chrono
