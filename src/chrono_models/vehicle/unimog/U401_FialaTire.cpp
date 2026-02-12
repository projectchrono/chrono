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

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_models/vehicle/unimog/U401_FialaTire.h"

namespace chrono {
namespace vehicle {
namespace unimog {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double U401_FialaTire::m_normalDamping = 7500;

const double U401_FialaTire::m_mass = 28.0;
const ChVector3d U401_FialaTire::m_inertia(2.5205, 4.8683, 2.5205);

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

    // load the vertical stiffness table
    m_vert_map.AddPoint(0, 0);
    m_vert_map.AddPoint(0.04, 14854.63356);
    m_vert_map.AddPoint(0.08, 29709.26711);
    m_vert_map.AddPoint(0.12, 44563.90067);
    m_vert_map.AddPoint(0.16, 59418.53422);

    m_max_depth = 0.160;
    m_max_val = 59418.53422;
    m_slope = 2.0 * (59418.53422 - 44563.90067) / 0.04;
}

double U401_FialaTire::GetNormalStiffnessForce(double depth) const {
    if (depth > m_max_depth) {
        // Linear extrapolation beyond available depth data
        return m_max_val + m_slope * (depth - m_max_depth);
    }

    // Return interpolated data
    return m_vert_map.GetVal(depth);
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
