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
// Sedan TMsimple tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/sedan/Sedan_TMsimpleTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace sedan {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string Sedan_TMsimpleTire::m_meshFile = "sedan/sedan_tire.obj";

const double Sedan_TMsimpleTire::m_mass = 11.5;
const ChVector<> Sedan_TMsimpleTire::m_inertia(0.156, 0.679, 0.156);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Sedan_TMsimpleTire::Sedan_TMsimpleTire(const std::string& name) : ChTMsimpleTire(name) {
    SetTMsimpleParams();
    if(GetName().compare("FL") == 0) {
        WritePlots("sedan.plt", "sedan");
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Sedan_TMsimpleTire::SetTMsimpleParams() {
    const double in2m = 0.0254;

    // Tire 245/40R18
    // mass ~11.5 kg
    // weight per tire aprox. 1600 lbf -> LI = 97
    // tire pressure 32 psi ~220000 N/m2
    // max pressure 50 psi ~350000 N/m2
    unsigned int li = 97;
    double w = 0.245;
    double r = 0.40;
    double rimdia = 18.0 * in2m;
    double pres_li = 350000;
    double pres_use = 220000;

    GuessPassCar70Par(li,       // tire load index []
                      w,        // tire width [m]
                      r,        // aspect ratio []
                      rimdia,   // rim diameter [m],
                      pres_li,  // infl. pressure for load index
                      pres_use  // infl. pressure for usage
    );
}

void Sedan_TMsimpleTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/245.40R18_" + GetName() + ".gpl";
    WritePlots(filename, "245.40R18");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Sedan_TMsimpleTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile,   // left side
                                               m_meshFile);  // right side
    } else {
        ChTMsimpleTire::AddVisualizationAssets(vis);
    }
}

void Sedan_TMsimpleTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMsimpleTire::RemoveVisualizationAssets();
}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono

