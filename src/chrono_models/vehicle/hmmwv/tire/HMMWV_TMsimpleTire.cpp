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

#include "chrono_models/vehicle/hmmwv/tire/HMMWV_TMsimpleTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string HMMWV_TMsimpleTire::m_meshFile_left = "hmmwv/hmmwv_tire_left.obj";
const std::string HMMWV_TMsimpleTire::m_meshFile_right = "hmmwv/hmmwv_tire_right.obj";

const double HMMWV_TMsimpleTire::m_mass = 37.6;
const ChVector<> HMMWV_TMsimpleTire::m_inertia(3.84, 6.69, 3.84);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_TMsimpleTire::HMMWV_TMsimpleTire(const std::string& name) : ChTMsimpleTire(name) {
    SetTMsimpleParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_TMsimpleTire::SetTMsimpleParams() {
    // Tire Size = 37 x 12.5 x 16.5 Load Range D
    // Tire Load 3850 lbs at 50 psi (Goodyear Military Tire Brochure 6th Edition)
    const double lbs2N = 4.4482216153;
    ////unsigned int li = 108;  // guessed from load spec. of the vehicle
    const double in2m = 0.0254;
    double h = (37.0 - 16.5) * in2m / 2.0;
    double w = 12.5 * in2m;
    double r = h / w;
    double rimdia = 16.5 * in2m;

    double load = 3850.0 * lbs2N;

    GuessTruck80Par(load,   // tire load [N]
                    w,      // tire width [m]
                    r,      // aspect ratio []
                    rimdia  // rim diameter [m]
    );
}

void HMMWV_TMsimpleTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "37x12.5x16.5_" + GetName() + ".gpl";
    WritePlots(filename, "37x12.5x16.5");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_TMsimpleTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMsimpleTire::AddVisualizationAssets(vis);
    }
}

void HMMWV_TMsimpleTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMsimpleTire::RemoveVisualizationAssets();
}

}  // namespace unimog
}  // end namespace vehicle
}  // end namespace chrono


