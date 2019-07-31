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
// Authors: Radu Serban, Justin Madsen, Asher Elmquist, Evan Hoerl
// =============================================================================
//
// CityBus wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/citybus/CityBus_Wheel.h"

namespace chrono {
namespace vehicle {
namespace citybus {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double CityBus_Wheel::m_mass = 30.0;
const ChVector<> CityBus_Wheel::m_inertia(.6, .63, .6);

const double CityBus_Wheel::m_radius = 0.33365;
const double CityBus_Wheel::m_width = 0.205;

const std::string CityBus_WheelLeft::m_meshName = "wheel_L_POV_geom";
const std::string CityBus_WheelLeft::m_meshFile = "citybus/CityBusRim_Right.obj";

const std::string CityBus_WheelRight::m_meshName = "wheel_R_POV_geom";
const std::string CityBus_WheelRight::m_meshFile = "citybus/CityBusRim_Left.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
CityBus_Wheel::CityBus_Wheel(const std::string& name) : ChWheel(name) {}

CityBus_WheelLeft::CityBus_WheelLeft(const std::string& name) : CityBus_Wheel(name) {}

CityBus_WheelRight::CityBus_WheelRight(const std::string& name) : CityBus_Wheel(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void CityBus_Wheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(GetMeshFile(), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetStatic(true);
        m_trimesh_shape->SetName(GetMeshName());
        m_spindle->AddAsset(m_trimesh_shape);
    } else {
        ChWheel::AddVisualizationAssets(vis);
    }
}

void CityBus_Wheel::RemoveVisualizationAssets() {
    ChWheel::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by CityBus_Wheel::AddVisualizationAssets.
    // This is important for the ChWheel object because a tire may add its own assets
    // to the same body (the spindle).
    auto it = std::find(m_spindle->GetAssets().begin(), m_spindle->GetAssets().end(), m_trimesh_shape);
    if (it != m_spindle->GetAssets().end())
        m_spindle->GetAssets().erase(it);
}

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
