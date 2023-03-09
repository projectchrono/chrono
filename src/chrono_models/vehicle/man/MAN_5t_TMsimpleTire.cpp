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
// MAN5t TMsimple subsystem
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/man/MAN_5t_TMsimpleTire.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double MAN_5t_TMsimpleTire::m_normalDamping = 7500;

const double MAN_5t_TMsimpleTire::m_mass = 37.6;
const ChVector<> MAN_5t_TMsimpleTire::m_inertia(3.84, 6.69, 3.84);

const std::string MAN_5t_TMsimpleTire::m_meshFile_left = "MAN_Kat1/meshes/MAN_tire.obj";
const std::string MAN_5t_TMsimpleTire::m_meshFile_right = "MAN_Kat1/meshes/MAN_tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MAN_5t_TMsimpleTire::MAN_5t_TMsimpleTire(const std::string& name) : ChTMsimpleTire(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void MAN_5t_TMsimpleTire::SetTMsimpleParams() {
    const double in2m = 0.0254;
    double h = (1.258 - 20 * in2m) / 2.0;
    double w = 0.385;
    double r = h / w;
    double rimdia = 20.0 * in2m;
    double pinfl_li = 760.0 * 1000;
    double pinfl_use = 450.0 * 1000;

    double load = 49050;

    GuessTruck80Par(load,    // tire load [N]
                    w,       // tire width [m]
                    r,       // aspect ratio []
                    rimdia,  // rim diameter [m]
                    pinfl_li, pinfl_use);

}

double MAN_5t_TMsimpleTire::GetNormalStiffnessForce(double depth) const {
    return depth*m_Cz;
}

double MAN_5t_TMsimpleTire::GetNormalDampingForce(double depth, double velocity) const {
    return m_normalDamping * velocity;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void MAN_5t_TMsimpleTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMsimpleTire::AddVisualizationAssets(vis);
    }
}

void MAN_5t_TMsimpleTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMsimpleTire::RemoveVisualizationAssets();
}

}  // end namespace man
}  // end namespace vehicle
}  // end namespace chrono

