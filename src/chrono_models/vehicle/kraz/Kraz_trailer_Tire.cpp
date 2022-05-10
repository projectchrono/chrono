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
// Krone ProfiLiner SP5 TMeasy tire subsystem 385/65R22.5 160
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/kraz/Kraz_trailer_Tire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace kraz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string Kraz_trailer_Tire::m_meshFile = "longhaul/meshes/SemiTrailer_tire.obj";

const double Kraz_trailer_Tire::m_mass = 69.3;
const ChVector<> Kraz_trailer_Tire::m_inertia(9.41035, 16.326, 9.41035);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Kraz_trailer_Tire::Kraz_trailer_Tire(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Kraz_trailer_Tire::SetTMeasyParams() {
    unsigned int li = 164;
    const double in2m = 0.0254;
    double w = 0.385;
    double r = 0.65;
    double rimdia = 22.5 * in2m;
    double pinfl_li = 1000.0 * 1000;
    double pinfl_use = 1000.0 * 1000;

    GuessTruck80Par(li,      // tire load index
                    w,       // tire width [m]
                    r,       // aspect ratio []
                    rimdia,  // rim diameter [m]
                    pinfl_li, pinfl_use);
}

void Kraz_trailer_Tire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/385_65R22p5_" + GetName() + ".gpl";
    WritePlots(filename, "385_65R22p5");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Kraz_trailer_Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void Kraz_trailer_Tire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
