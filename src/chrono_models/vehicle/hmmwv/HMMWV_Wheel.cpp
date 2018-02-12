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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// HMMWV wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double HMMWV_Wheel::m_mass = 18.8;
const ChVector<> HMMWV_Wheel::m_inertia(0.113, 0.113, 0.113);

const double HMMWV_Wheel::m_radius = 0.268;
const double HMMWV_Wheel::m_width = 0.22;

const std::string HMMWV_WheelLeft::m_meshName = "wheel_L_POV_geom";
const std::string HMMWV_WheelLeft::m_meshFile = "hmmwv/wheel_L.obj";

const std::string HMMWV_WheelRight::m_meshName = "wheel_R_POV_geom";
const std::string HMMWV_WheelRight::m_meshFile = "hmmwv/wheel_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_Wheel::HMMWV_Wheel(const std::string& name) : ChWheel(name) {}

HMMWV_WheelLeft::HMMWV_WheelLeft(const std::string& name) : HMMWV_Wheel(name) {}

HMMWV_WheelRight::HMMWV_WheelRight(const std::string& name) : HMMWV_Wheel(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_Wheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(GetMeshFile(), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(GetMeshName());
        m_spindle->AddAsset(m_trimesh_shape);
    } else {
        ChWheel::AddVisualizationAssets(vis);
    }
}

void HMMWV_Wheel::RemoveVisualizationAssets() {
    ChWheel::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by HMMWV_Wheel::AddVisualizationAssets.
    // This is important for the ChWheel object because a tire may add its own assets
    // to the same body (the spindle).
    auto it = std::find(m_spindle->GetAssets().begin(), m_spindle->GetAssets().end(), m_trimesh_shape);
    if (it != m_spindle->GetAssets().end())
        m_spindle->GetAssets().erase(it);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
