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
// HMMWV LuGre subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_LugreTire.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;

const double HMMWV_LugreTire::m_radius = 18.15 * in2m;
const double HMMWV_LugreTire::m_mass = 37.6;
const ChVector<> HMMWV_LugreTire::m_inertia(3.84, 6.69, 3.84);
const double HMMWV_LugreTire::m_discLocs[] = {-5 * in2m, 0 * in2m, 5 * in2m};

const double HMMWV_LugreTire::m_normalStiffness = 326332;
const double HMMWV_LugreTire::m_normalDamping = 348;

const std::string HMMWV_LugreTire::m_meshName = "hmmwv_tire_POV_geom";
const std::string HMMWV_LugreTire::m_meshFile = "hmmwv/hmmwv_tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_LugreTire::HMMWV_LugreTire(const std::string& name) : ChLugreTire(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_LugreTire::SetLugreParams() {
    // Longitudinal direction
    m_sigma0[0] = 181;
    m_sigma1[0] = 1;
    m_sigma2[0] = 0.02;

    m_Fc[0] = 0.75;
    m_Fs[0] = 0.84;

    m_vs[0] = 0.035;

    // Lateral direction
    m_sigma0[1] = 60;
    m_sigma1[1] = 0.1;
    m_sigma2[1] = 0.002;

    m_Fc[1] = 0.75;
    m_Fs[1] = 0.84;

    m_vs[1] = 0.035;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_LugreTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_trimesh_shape->SetStatic(true);
        m_wheel->AddAsset(m_trimesh_shape);
    } else {
        ChLugreTire::AddVisualizationAssets(vis);
    }
}

void HMMWV_LugreTire::RemoveVisualizationAssets() {
    ChLugreTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by HMMWV_LugreTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_trimesh_shape);
    if (it != m_wheel->GetAssets().end())
        m_wheel->GetAssets().erase(it);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
