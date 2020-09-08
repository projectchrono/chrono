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

#include "subsystems/SemiTractor_tire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string SemiTractor_tire::m_meshFile = "longhaul/meshes/SemiTractor_tire.obj";

const double SemiTractor_tire::m_mass = 90.0;
const chrono::ChVector<> SemiTractor_tire::m_inertia(11.5173, 20.9672, 11.5173);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SemiTractor_tire::SemiTractor_tire(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SemiTractor_tire::SetTMeasyParams() {
    const double lbs2N = 4.4482216153;
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

void SemiTractor_tire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/12R20_" + GetName() + ".gpl";
    WritePlots(filename, "12R20");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SemiTractor_tire::AddVisualizationAssets(chrono::vehicle::VisualizationType vis) {
    if (vis == chrono::vehicle::VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void SemiTractor_tire::RemoveVisualizationAssets() {
    ChTMeasyTire::RemoveVisualizationAssets();
    RemoveVisualizationMesh(m_trimesh_shape);
}
