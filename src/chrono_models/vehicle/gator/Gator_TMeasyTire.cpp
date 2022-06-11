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
// Authors: Radu Serban
// =============================================================================
//
// Gator TMeasy tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/gator/Gator_TMeasyTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace gator {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Gator_TMeasyTire_Front::m_mass = 9.3;
const ChVector<> Gator_TMeasyTire_Front::m_inertia(0.258, 0.416, 0.258);
const std::string Gator_TMeasyTire_Front::m_meshFile_left = "gator/gator_wheel_FL.obj";
const std::string Gator_TMeasyTire_Front::m_meshFile_right = "gator/gator_wheel_FR.obj";

const double Gator_TMeasyTire_Rear::m_mass = 9.3;
const ChVector<> Gator_TMeasyTire_Rear::m_inertia(0.258, 0.416, 0.258);
const std::string Gator_TMeasyTire_Rear::m_meshFile_left = "gator/gator_wheel_RL.obj";
const std::string Gator_TMeasyTire_Rear::m_meshFile_right = "gator/gator_wheel_RR.obj";

// -----------------------------------------------------------------------------

Gator_TMeasyTire_Front::Gator_TMeasyTire_Front(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

void Gator_TMeasyTire_Front::SetTMeasyParams() {
    double wheel_diam = 0.5715;  // 22.5 inches
    double rim_diam = 0.1778;    // 7 inches
    double w = 0.254;            // 10 inches

    double h = 0.5 * (wheel_diam - rim_diam);
    double r = h / w;

    double load = 1800;

    GuessTruck80Par(load,      // tire load [N]
                    w,         // tire width [m]
                    r,         // aspect ratio []
                    rim_diam,  // rim diameter [m]
                    1.0, 1.0,  // inflation pressures at load index and use configuration
                    1.0        // damping ratio
    );

    SetFrictionCoefficient(0.6f);
}

void Gator_TMeasyTire_Front::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/22.5x10x8" + GetName() + ".gpl";
    WritePlots(filename, "22.5x10x8");
}

void Gator_TMeasyTire_Front::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void Gator_TMeasyTire_Front::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------

Gator_TMeasyTire_Rear::Gator_TMeasyTire_Rear(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

void Gator_TMeasyTire_Rear::SetTMeasyParams() {
    double wheel_diam = 0.635;  // 25 inches
    double rim_diam = 0.2286;   // 9 inches
    double w = 0.3048;          // 12 inches

    double h = 0.5 * (wheel_diam - rim_diam);
    double r = h / w;

    double load = 2600;

    GuessTruck80Par(load,      // tire load [N]
                    w,         // tire width [m]
                    r,         // aspect ratio []
                    rim_diam,  // rim diameter [m]
                    1.0, 1.0,  // inflation pressures at load index and use configuration
                    1.0        // damping ratio
    );

    SetFrictionCoefficient(0.6f);
}

void Gator_TMeasyTire_Rear::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/25x12x9" + GetName() + ".gpl";
    WritePlots(filename, "25x12x9");
}

void Gator_TMeasyTire_Rear::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void Gator_TMeasyTire_Rear::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono
