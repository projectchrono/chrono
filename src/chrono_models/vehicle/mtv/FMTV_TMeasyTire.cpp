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
// FMTV TMeasy tire subsystem 395/85R20 159G 655 kPa
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/mtv/FMTV_TMeasyTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string FMTV_TMeasyTire::m_meshFile = "mtv/meshes/MTV_tire.obj";

const double FMTV_TMeasyTire::m_mass = 110.0;
const ChVector<> FMTV_TMeasyTire::m_inertia(16.9, 29.5, 16.9);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FMTV_TMeasyTire::FMTV_TMeasyTire(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void FMTV_TMeasyTire::SetTMeasyParams() {
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

void FMTV_TMeasyTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/395_85R20_" + GetName() + ".gpl";
    WritePlots(filename, "395/85R20");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void FMTV_TMeasyTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void FMTV_TMeasyTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
