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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Sedan TMsimple subsystem
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/sedan/Sedan_TMsimpleTire.h"

namespace chrono {
namespace vehicle {
namespace sedan {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Sedan_TMsimpleTire::m_normalDamping = 7500;

const double Sedan_TMsimpleTire::m_mass = 11.5;
const ChVector<> Sedan_TMsimpleTire::m_inertia(0.156, 0.679, 0.156);

const std::string Sedan_TMsimpleTire::m_meshFile_left = "sedan/sedan_tire.obj";
const std::string Sedan_TMsimpleTire::m_meshFile_right = "sedan/sedan_tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Sedan_TMsimpleTire::Sedan_TMsimpleTire(const std::string& name) : ChTMsimpleTire(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void Sedan_TMsimpleTire::SetTMsimpleParams() {
    const double in2m = 0.0254;
    // Tire 245/40R18
    // mass ~11.5 kg
    // load per tire aprox. 1600 lbf -> LI = 97
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

double Sedan_TMsimpleTire::GetNormalStiffnessForce(double depth) const {
    return depth*m_Cz; // just linear in this example
}

double Sedan_TMsimpleTire::GetNormalDampingForce(double depth, double velocity) const {
    return m_normalDamping * velocity;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Sedan_TMsimpleTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
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


