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
// HMMWV Fiala subsystem
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/unimog/U401_FialaTire.h"

namespace chrono {
namespace vehicle {
namespace unimog {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double U401_FialaTire::m_normalDamping = 7500;

const double U401_FialaTire::m_mass = 28.0;
const ChVector<> U401_FialaTire::m_inertia(2.5205, 4.8683, 2.5205);

const std::string U401_FialaTire::m_meshFile_left = "unimog/U401_Tire.obj";
const std::string U401_FialaTire::m_meshFile_right = "unimog/U401_Tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
U401_FialaTire::U401_FialaTire(const std::string& name) : ChFialaTire(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void U401_FialaTire::SetFialaParams() {
    // Parameters were converted from TMsimple
    m_unloaded_radius = 0.4375;
    m_width = 0.184;
    m_rolling_resistance = 0.015;
    m_c_slip = 44254.1;
    m_c_alpha = 18949.2;
    m_u_min = 0.54397;
    m_u_max = 0.88468;
    m_relax_length_x = 2;
    m_relax_length_y = 2;

    // load the vertical stiffness table
    m_vert_map.AddPoint(0.000, 0);
    m_vert_map.AddPoint(0.005, 585);
    m_vert_map.AddPoint(0.010, 1286);
    m_vert_map.AddPoint(0.015, 2352);
    m_vert_map.AddPoint(0.020, 3477);
    m_vert_map.AddPoint(0.025, 4798);
    m_vert_map.AddPoint(0.030, 6190);
    m_vert_map.AddPoint(0.035, 7540);
    m_vert_map.AddPoint(0.040, 9027);
    m_vert_map.AddPoint(0.045, 10570);
    m_vert_map.AddPoint(0.050, 12139);
    m_vert_map.AddPoint(0.055, 13654);
    m_vert_map.AddPoint(0.060, 15368);
    m_vert_map.AddPoint(0.065, 16904);
    m_vert_map.AddPoint(0.070, 18469);
    m_vert_map.AddPoint(0.075, 20089);
    m_vert_map.AddPoint(0.080, 21699);

    m_max_depth = 0.080;
    m_max_val = 21699;
    m_slope = 3.22e5;
}

double U401_FialaTire::GetNormalStiffnessForce(double depth) const {
    if (depth > m_max_depth) {
        // Linear extrapolation beyond available depth data
        return m_max_val + m_slope * (depth - m_max_depth);
    }

    // Return interpolated data
    return m_vert_map.Get_y(depth);
}

double U401_FialaTire::GetNormalDampingForce(double depth, double velocity) const {
    return m_normalDamping * velocity;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void U401_FialaTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChFialaTire::AddVisualizationAssets(vis);
    }
}

void U401_FialaTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChFialaTire::RemoveVisualizationAssets();
}

}  // end namespace unimog
}  // end namespace vehicle
}  // end namespace chrono

