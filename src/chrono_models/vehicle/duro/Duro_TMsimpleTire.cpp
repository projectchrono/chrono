// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Duro TMsimple tire subsystem 275/80 R20 (= 10.5 R20)
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/duro/Duro_TMsimpleTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace duro {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string Duro_TMsimpleTire::m_meshFile_left = "duro/Duro_Tire.obj";
const std::string Duro_TMsimpleTire::m_meshFile_right = "duro/Duro_Tire.obj";

const double Duro_TMsimpleTire::m_mass = 34.4;
const ChVector<> Duro_TMsimpleTire::m_inertia(3.34, 6.28, 3.34);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Duro_TMsimpleTire::Duro_TMsimpleTire(const std::string& name) : ChTMsimpleTire(name) {
    SetTMsimpleParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Duro_TMsimpleTire::SetTMsimpleParams() {
    const double in2m = 0.0254;
    double w = 0.275;
    double r = 0.8;
    double rimdia = 20.0 * in2m;

    int li = 128;
    double load = GetTireMaxLoad(li);

    GuessTruck80Par(load,   // tire load [N]
                    w,      // tire width [m]
                    r,      // aspect ratio []
                    rimdia  // rim diameter [m]
    );
}

void Duro_TMsimpleTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/275_80R20" + GetName() + ".gpl";
    WritePlots(filename, "275/80R20 128K");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Duro_TMsimpleTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMsimpleTire::AddVisualizationAssets(vis);
    }
}

void Duro_TMsimpleTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMsimpleTire::RemoveVisualizationAssets();
}

}  // namespace duro
}  // end namespace vehicle
}  // end namespace chrono
