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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Kraz 64431 TMeasy tire subsystem 12.00R20 150
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/kraz/Kraz_tractor_Tire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace kraz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string Kraz_tractor_Tire::m_meshFile = "longhaul/meshes/SemiTractor_tire.obj";

const double Kraz_tractor_Tire::m_mass = 90.0;
const chrono::ChVector<> Kraz_tractor_Tire::m_inertia(11.5173, 20.9672, 11.5173);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Kraz_tractor_Tire::Kraz_tractor_Tire(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Kraz_tractor_Tire::SetTMeasyParams() {
    unsigned int li = 150;
    const double in2m = 0.0254;
    double w = 12.0 * in2m;
    double r = 1.0;
    double rimdia = 20.0 * in2m;
    double pinfl_li = 850.0 * 1000;
    double pinfl_use = 850.0 * 1000;

    GuessTruck80Par(li,      // tire load index
                    w,       // tire width [m]
                    r,       // aspect ratio []
                    rimdia,  // rim diameter [m]
                    pinfl_li, pinfl_use);
}

void Kraz_tractor_Tire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/12R20_" + GetName() + ".gpl";
    WritePlots(filename, "12R20");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Kraz_tractor_Tire::AddVisualizationAssets(chrono::vehicle::VisualizationType vis) {
    if (vis == chrono::vehicle::VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void Kraz_tractor_Tire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
