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
// mrole TMeasy tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/mrole/mrole_TMeasyTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// -----------------------------------------------------------------------------
// Static variables onroad
// -----------------------------------------------------------------------------

const std::string mrole_TMeasyTire::m_meshFile = "mrole/meshes/mrole_tire.obj";

const double mrole_TMeasyTire::m_mass = 105.0;
const ChVector<> mrole_TMeasyTire::m_inertia(21.72, 38.74, 21.72);

// -----------------------------------------------------------------------------
// Static variables offroad soil
// -----------------------------------------------------------------------------

const std::string mrole_TMeasyTireSoil::m_meshFile = "mrole/meshes/mrole_tire.obj";

const double mrole_TMeasyTireSoil::m_mass = 105.0;
const ChVector<> mrole_TMeasyTireSoil::m_inertia(21.72, 38.74, 21.72);

// -----------------------------------------------------------------------------
// Static variables offroad sand
// -----------------------------------------------------------------------------

const std::string mrole_TMeasyTireSand::m_meshFile = "mrole/meshes/mrole_tire.obj";

const double mrole_TMeasyTireSand::m_mass = 105.0;
const ChVector<> mrole_TMeasyTireSand::m_inertia(21.72, 38.74, 21.72);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
mrole_TMeasyTire::mrole_TMeasyTire(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void mrole_TMeasyTire::SetTMeasyParams() {
    // Tire Size = 37 x 12.5 x 16.5 Load Range D
    // Tire Load 3850 lbs at 50 psi (Goodyear Military Tire Brochure 6th Edition)

    ////unsigned int li = 108;  // guessed from load spec. of the vehicle
    const double in2m = 0.0254;
    double w = 0.415;
    double r = 0.8;
    ////double h = r * w;
    double rimdia = 27.0 * in2m;

    double load = 9.81 * 4500.0;

    GuessTruck80Par(load,    // tire load [N]
                    w,       // tire width [m]
                    r,       // aspect ratio []
                    rimdia,  // rim diameter [m]
                    6.7,     // inflation pressure at design load
                    6.7      // inflation pressure at use
    );
}

void mrole_TMeasyTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/37x12.5x16.5_" + GetName() + ".gpl";
    WritePlots(filename, "37x12.5x16.5");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void mrole_TMeasyTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void mrole_TMeasyTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
mrole_TMeasyTireSoil::mrole_TMeasyTireSoil(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void mrole_TMeasyTireSoil::SetTMeasyParams() {
    // Tire Size = 37 x 12.5 x 16.5 Load Range D
    // Tire Load 3850 lbs at 50 psi (Goodyear Military Tire Brochure 6th Edition)

    ////unsigned int li = 108;  // guessed from load spec. of the vehicle
    const double in2m = 0.0254;
    double w = 0.415;
    double r = 0.8;
    ////double h = r * w;
    double rimdia = 27.0 * in2m;

    double load = 9.81 * 4500.0;

    GuessTruck80Par(load,    // tire load [N]
                    w,       // tire width [m]
                    r,       // aspect ratio []
                    rimdia,  // rim diameter [m]
                    6.7,     // inflation pressure at design load
                    3.9      // inflation pressure at use
    );
}

void mrole_TMeasyTireSoil::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/37x12.5x16.5_" + GetName() + ".gpl";
    WritePlots(filename, "37x12.5x16.5");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void mrole_TMeasyTireSoil::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void mrole_TMeasyTireSoil::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
mrole_TMeasyTireSand::mrole_TMeasyTireSand(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void mrole_TMeasyTireSand::SetTMeasyParams() {
    // Tire Size = 37 x 12.5 x 16.5 Load Range D
    // Tire Load 3850 lbs at 50 psi (Goodyear Military Tire Brochure 6th Edition)

    ////unsigned int li = 108;  // guessed from load spec. of the vehicle
    const double in2m = 0.0254;
    double w = 0.415;
    double r = 0.8;
    ////double h = r * w;
    double rimdia = 27.0 * in2m;

    double load = 9.81 * 4500.0;

    GuessTruck80Par(load,    // tire load [N]
                    w,       // tire width [m]
                    r,       // aspect ratio []
                    rimdia,  // rim diameter [m]
                    6.7,     // inflation pressure at design load
                    2.5      // inflation pressure at use
    );
}

void mrole_TMeasyTireSand::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/37x12.5x16.5_" + GetName() + ".gpl";
    WritePlots(filename, "37x12.5x16.5");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void mrole_TMeasyTireSand::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void mrole_TMeasyTireSand::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
