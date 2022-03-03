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
// MAN Kat TMeasy tire subsystem 14.00R20 450 MPa
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/man/MAN_5t_TMeasyTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string MAN_5t_TMeasyTire::m_meshFile = "MAN_Kat1/meshes/MAN_tire.obj";

const double MAN_5t_TMeasyTire::m_mass = 104.0;
const ChVector<> MAN_5t_TMeasyTire::m_inertia(17.8651, 31.6623, 17.8651);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MAN_5t_TMeasyTire::MAN_5t_TMeasyTire(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void MAN_5t_TMeasyTire::SetTMeasyParams() {
    ////unsigned int li = 164;
    const double in2m = 0.0254;
    double h = (1.258 - 20 * in2m) / 2.0;
    double w = 0.385;
    double r = h / w;
    double rimdia = 20.0 * in2m;
    double pinfl_li = 760.0 * 1000;
    double pinfl_use = 450.0 * 1000;

    double load = 49050;

    GuessTruck80Par(load,    // tire load [N]
                    w,       // tire width [m]
                    r,       // aspect ratio []
                    rimdia,  // rim diameter [m]
                    pinfl_li, pinfl_use);
}

void MAN_5t_TMeasyTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/14.00R20_" + GetName() + ".gpl";
    WritePlots(filename, "14.00R20");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void MAN_5t_TMeasyTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,    // left side
                                               m_meshFile);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void MAN_5t_TMeasyTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
