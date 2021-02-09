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
// Authors: Radu Serban, Justin Madsen, Jayne Henry
// =============================================================================
//
// RCCar wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/rccar/RCCar_Wheel.h"

using namespace chrono::vehicle;
using namespace chrono;

namespace chrono {
namespace vehicle {
namespace rccar {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;

const double RCCar_Wheel::m_mass = lb2kg * 1.0;
const ChVector<> RCCar_Wheel::m_inertia(0.001366, 0.001561, 0.001366);

const double RCCar_Wheel::m_radius = in2m * 2.22525;
const double RCCar_Wheel::m_width = in2m * 3.3;

const std::string RCCar_WheelLeft::m_meshName = "wheel_L_POV_geom";
// const std::string RCCar_WheelLeft::m_meshFile = "rccar/wheel_L.obj";
const std::string RCCar_WheelLeft::m_meshFile = "rccar/wheel_L.obj";

const std::string RCCar_WheelRight::m_meshName = "wheel_R_POV_geom";
const std::string RCCar_WheelRight::m_meshFile = "rccar/wheel_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
RCCar_Wheel::RCCar_Wheel(const std::string& name) : ChWheel(name) {}

RCCar_WheelLeft::RCCar_WheelLeft(const std::string& name) : RCCar_Wheel(name) {}

RCCar_WheelRight::RCCar_WheelRight(const std::string& name) : RCCar_Wheel(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RCCar_Wheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(GetMeshFile(), false, false);
        m_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(GetMeshName());
        m_trimesh_shape->SetStatic(true);
        m_spindle->AddAsset(m_trimesh_shape);
    } else {
        ChWheel::AddVisualizationAssets(vis);
    }
}

void RCCar_Wheel::RemoveVisualizationAssets() {
    ChWheel::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by RCCar_Wheel::AddVisualizationAssets.
    // This is important for the ChWheel object because a tire may add its own assets
    // to the same body (the spindle).
    auto it = std::find(m_spindle->GetAssets().begin(), m_spindle->GetAssets().end(), m_trimesh_shape);
    if (it != m_spindle->GetAssets().end())
        m_spindle->GetAssets().erase(it);
}

}  // end namespace rccar
}  // namespace vehicle
}  // namespace chrono
