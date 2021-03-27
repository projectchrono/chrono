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
// Authors: Rainer Gericke, Asher Elmquist
// =============================================================================
//
// Sedan TMeasy tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/sedan/Sedan_TMeasyTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace sedan {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string Sedan_TMeasyTire::m_meshFile = "sedan/sedan_tire.obj";

const double Sedan_TMeasyTire::m_mass = 37.6;
const ChVector<> Sedan_TMeasyTire::m_inertia(3.84, 6.69, 3.84);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Sedan_TMeasyTire::Sedan_TMeasyTire(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Sedan_TMeasyTire::SetTMeasyParams() {
    // Tire Size = 37 x 12.5 x 16.5 Load Range D
    // weight per tire aprox. 10000 N -> LI = 108

    unsigned int li = 121;
    const double in2m = 0.0254;
    double w = 0.225;
    double r = 0.75;
    double rimdia = 16.0 * in2m;
    double pres_li = 590000;
    double pres_use = 220000;

    GuessTruck80Par(li,       // tire load index []
                    w,        // tire width [m]
                    r,        // aspect ratio []
                    rimdia,   // rim diameter [m],
                    pres_li,  // infl. pressure for load index
                    pres_use  // infl. pressure for usage
    );
}

void Sedan_TMeasyTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/37x12.5x16.5_" + GetName() + ".gpl";
    WritePlots(filename, "37x12.5x16.5");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Sedan_TMeasyTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void Sedan_TMeasyTire::RemoveVisualizationAssets() {
    ChTMeasyTire::RemoveVisualizationAssets();
    RemoveVisualizationMesh(m_trimesh_shape);
}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
