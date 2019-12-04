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
#include "chrono_models/vehicle/hmmwv/HMMWV_FialaTire.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double HMMWV_FialaTire::m_normalDamping = 7500;

const double HMMWV_FialaTire::m_mass = 37.6;
const ChVector<> HMMWV_FialaTire::m_inertia(3.84, 6.69, 3.84);

const std::string HMMWV_FialaTire::m_meshName = "hmmwv_tire_POV_geom";
const std::string HMMWV_FialaTire::m_meshFile = "hmmwv/hmmwv_tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_FialaTire::HMMWV_FialaTire(const std::string& name) : ChFialaTire(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void HMMWV_FialaTire::SetFialaParams() {
    // Parameters were roughly converted from TMeasy, still questionable, but better
    m_unloaded_radius = 0.47;
    m_width = 0.318;
    m_rolling_resistance = 0.015;
    m_c_slip = 193929.0;
    m_c_alpha = 50000.0;
    m_u_min = 0.5568;
    m_u_max = 0.9835;
    // m_relax_length_x = 0.1244;
    // m_relax_length_y = 0.0317;
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

double HMMWV_FialaTire::GetNormalStiffnessForce(double depth) const {
    if (depth > m_max_depth) {
        // Linear extrapolation beyond available depth data
        return m_max_val + m_slope * (depth - m_max_depth);
    }

    // Return interpolated data
    return m_vert_map.Get_y(depth);
}

double HMMWV_FialaTire::GetNormalDampingForce(double depth, double velocity) const {
    return m_normalDamping * velocity;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_FialaTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        trimesh->Transform(ChVector<>(0, GetOffset(), 0), ChMatrix33<>(1));
        m_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_trimesh_shape->SetStatic(true);
        m_wheel->GetSpindle()->AddAsset(m_trimesh_shape);
    } else {
        ChFialaTire::AddVisualizationAssets(vis);
    }
}

void HMMWV_FialaTire::RemoveVisualizationAssets() {
    ChFialaTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by HMMWV_FialaTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto& assets = m_wheel->GetSpindle()->GetAssets();
    auto it = std::find(assets.begin(), assets.end(), m_trimesh_shape);
    if (it != assets.end())
        assets.erase(it);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
