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
// MAN TMsimple tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/man/tire/MAN_TMsimpleTire.h"
#include "chrono_vehicle/ChVehicleDataPath.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string MAN_TMsimpleTire::m_meshFile_left = "MAN/meshes/MAN_tire.obj";
const std::string MAN_TMsimpleTire::m_meshFile_right = "MAN/meshes/MAN_tire.obj";

const double MAN_TMsimpleTire::m_mass = 104.0;
const ChVector3d MAN_TMsimpleTire::m_inertia(17.86, 31.66, 17.86);

// -----------------------------------------------------------------------------

MAN_TMsimpleTire::MAN_TMsimpleTire(const std::string& name) : ChTMsimpleTire(name) {
    SetTMsimpleParams();
}

// -----------------------------------------------------------------------------

void MAN_TMsimpleTire::SetTMsimpleParams() {
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

void MAN_TMsimpleTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "14.00R20_" + GetName() + ".gpl";
    WritePlots(filename, "14.00R20");
}

// -----------------------------------------------------------------------------

void MAN_TMsimpleTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMsimpleTire::AddVisualizationAssets(vis);
    }
}

void MAN_TMsimpleTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMsimpleTire::RemoveVisualizationAssets();
}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
