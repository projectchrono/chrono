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
// U401 TMeasy tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/unimog/U401_TMsimpleTire.h"
#include "chrono_vehicle/ChVehicleDataPath.h"

namespace chrono {
namespace vehicle {
namespace unimog {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string U401_TMsimpleTire::m_meshFile_left = "unimog/U401_Tire.obj";
const std::string U401_TMsimpleTire::m_meshFile_right = "unimog/U401_Tire.obj";

const double U401_TMsimpleTire::m_mass = 28.0;
const ChVector3d U401_TMsimpleTire::m_inertia(2.5205, 4.8683, 2.5205);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
U401_TMsimpleTire::U401_TMsimpleTire(const std::string& name) : ChTMsimpleTire(name) {
    SetTMsimpleParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void U401_TMsimpleTire::SetTMsimpleParams() {
    // Tire Size = 6.50-20

    const double lbs2N = 4.4482216153;
    ////unsigned int li = 108;  // guessed from load spec. of the vehicle
    const double in2m = 0.0254;
    double h = 6.5 * in2m;
    double w = 6.5 * in2m;
    double r = h / w;
    double rimdia = 20.0 * in2m;

    double load = 1125.0 * lbs2N;

    GuessTruck80Par(load,   // tire load [N]
                    w,      // tire width [m]
                    r,      // aspect ratio []
                    rimdia  // rim diameter [m]
    );
}

void U401_TMsimpleTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/6.5-20" + GetName() + ".gpl";
    WritePlots(filename, "6.5-20");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void U401_TMsimpleTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMsimpleTire::AddVisualizationAssets(vis);
    }
}

void U401_TMsimpleTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMsimpleTire::RemoveVisualizationAssets();
}

}  // namespace unimog
}  // end namespace vehicle
}  // end namespace chrono
