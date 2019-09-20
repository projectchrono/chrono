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
// HMMWV Pacejka 2002 tire subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_PacejkaTire.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double HMMWV_PacejkaTire::m_mass = 37.6;
const ChVector<> HMMWV_PacejkaTire::m_inertia(3.84, 6.69, 3.84);

const std::string HMMWV_PacejkaTire::m_pacTireFile = "hmmwv/tire/HMMWV_pacejka.tir";

const std::string HMMWV_PacejkaTire::m_meshName = "hmmwv_tire_POV_geom";
const std::string HMMWV_PacejkaTire::m_meshFile = "hmmwv/hmmwv_tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_PacejkaTire::HMMWV_PacejkaTire(const std::string& name)
    : ChPacejkaTire(name, vehicle::GetDataFile(m_pacTireFile)) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_PacejkaTire::AddVisualizationAssets(VisualizationType vis) {
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
        ChPacejkaTire::AddVisualizationAssets(vis);
    }
}

void HMMWV_PacejkaTire::RemoveVisualizationAssets() {
    ChPacejkaTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by HMMWV_PacejkaTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets to the
    // same body (the spindle/wheel).
    auto& assets = m_wheel->GetSpindle()->GetAssets();
    auto it = std::find(assets.begin(), assets.end(), m_trimesh_shape);
    if (it != assets.end())
        assets.erase(it);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
