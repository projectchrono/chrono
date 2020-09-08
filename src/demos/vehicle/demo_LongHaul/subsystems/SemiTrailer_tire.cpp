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

#include "subsystems/SemiTrailer_tire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string SemiTrailer_tire::m_meshFile = "longhaul/meshes/SemiTrailer_tire.obj";

const double SemiTrailer_tire::m_mass = 69.3;
const chrono::ChVector<> SemiTrailer_tire::m_inertia(9.41035, 16.326, 9.41035);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SemiTrailer_tire::SemiTrailer_tire(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SemiTrailer_tire::SetTMeasyParams() {
    const double lbs2N = 4.4482216153;
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

void SemiTrailer_tire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/385_65R22p5_" + GetName() + ".gpl";
    WritePlots(filename, "385_65R22p5");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SemiTrailer_tire::AddVisualizationAssets(chrono::vehicle::VisualizationType vis) {
    if (vis == chrono::vehicle::VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void SemiTrailer_tire::RemoveVisualizationAssets() {
    ChTMeasyTire::RemoveVisualizationAssets();
    RemoveVisualizationMesh(m_trimesh_shape);
}
