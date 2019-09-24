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
// UAZBUS wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/uaz/UAZBUS_Wheel.h"

namespace chrono {
namespace vehicle {
namespace uaz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double UAZBUS_Wheel::m_mass = 12.0;
const ChVector<> UAZBUS_Wheel::m_inertia(0.240642, 0.410903, 0.240642);

const double UAZBUS_Wheel::m_radius = 0.2032;
const double UAZBUS_Wheel::m_width = 0.1524;

const std::string UAZBUS_WheelLeft::m_meshName = "wheel_L_POV_geom";
const std::string UAZBUS_WheelLeft::m_meshFile = "uaz/wheel_L.obj";

const std::string UAZBUS_WheelRight::m_meshName = "wheel_R_POV_geom";
const std::string UAZBUS_WheelRight::m_meshFile = "uaz/wheel_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
UAZBUS_Wheel::UAZBUS_Wheel(const std::string& name) : ChWheel(name) {}

UAZBUS_WheelLeft::UAZBUS_WheelLeft(const std::string& name) : UAZBUS_Wheel(name) {}

UAZBUS_WheelRight::UAZBUS_WheelRight(const std::string& name) : UAZBUS_Wheel(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void UAZBUS_Wheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(GetMeshFile(), false, false);
        trimesh->Transform(ChVector<>(0, m_offset, 0), ChMatrix33<>(1));
        m_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(GetMeshName());
        m_trimesh_shape->SetStatic(true);
        GetSpindle()->AddAsset(m_trimesh_shape);
    } else {
        ChWheel::AddVisualizationAssets(vis);
    }
}

void UAZBUS_Wheel::RemoveVisualizationAssets() {
    ChWheel::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by UAZBUS_Wheel::AddVisualizationAssets.
    // This is important for the ChWheel object because a tire may add its own assets
    // to the same body (the spindle).
    auto it = std::find(GetSpindle()->GetAssets().begin(), GetSpindle()->GetAssets().end(), m_trimesh_shape);
    if (it != GetSpindle()->GetAssets().end())
        GetSpindle()->GetAssets().erase(it);
}

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
