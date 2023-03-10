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
// HMMWV TMsimple subsystem
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_TMsimpleTire.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double HMMWV_TMsimpleTire::m_normalDamping = 7500;

const double HMMWV_TMsimpleTire::m_mass = 37.6;
const ChVector<> HMMWV_TMsimpleTire::m_inertia(3.84, 6.69, 3.84);

const std::string HMMWV_TMsimpleTire::m_meshFile_left = "hmmwv/hmmwv_tire_left.obj";
const std::string HMMWV_TMsimpleTire::m_meshFile_right = "hmmwv/hmmwv_tire_right.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_TMsimpleTire::HMMWV_TMsimpleTire(const std::string& name) : ChTMsimpleTire(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void HMMWV_TMsimpleTire::SetTMsimpleParams() {
    // Parameters were roughly converted from TMeasy, still questionable, but better
    const double lbs2N = 4.4482216153;
    ////unsigned int li = 108;  // guessed from load spec. of the vehicle
    const double in2m = 0.0254;
    double h = (37.0 - 16.5) * in2m / 2.0;
    double w = 12.5 * in2m;
    double r = h / w;
    double rimdia = 16.5 * in2m;

    double load = 3850.0 * lbs2N;

    ChTMsimpleTire::GuessTruck80Par(load, w, r, rimdia);
    
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

double HMMWV_TMsimpleTire::GetNormalStiffnessForce(double depth) const {
    if (depth > m_max_depth) {
        // Linear extrapolation beyond available depth data
        return m_max_val + m_slope * (depth - m_max_depth);
    }
    // Return interpolated data
    return m_vert_map.Get_y(depth);
}

double HMMWV_TMsimpleTire::GetNormalDampingForce(double depth, double velocity) const {
    return m_normalDamping * velocity;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_TMsimpleTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMsimpleTire::AddVisualizationAssets(vis);
    }
}

void HMMWV_TMsimpleTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMsimpleTire::RemoveVisualizationAssets();
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

