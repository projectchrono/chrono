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
// Authors: Radu Serban, Justin Madsen, Asher Elmquist
// =============================================================================
//
// Sedan wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/sedan/Sedan_Wheel.h"

namespace chrono {
namespace vehicle {
namespace sedan {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Sedan_Wheel::m_mass = 11.0;
const ChVector<> Sedan_Wheel::m_inertia(0.100, 0.100, 0.100);

const double Sedan_Wheel::m_radius = 0.3365;
const double Sedan_Wheel::m_width = 0.205;

const std::string Sedan_WheelLeft::m_meshName = "wheel_L_POV_geom";
const std::string Sedan_WheelLeft::m_meshFile = "sedan/wheel_hub_right.obj";

const std::string Sedan_WheelRight::m_meshName = "wheel_R_POV_geom";
const std::string Sedan_WheelRight::m_meshFile = "sedan/wheel_hub_left.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Sedan_Wheel::Sedan_Wheel(const std::string& name) : ChWheel(name) {}

Sedan_WheelLeft::Sedan_WheelLeft(const std::string& name) : Sedan_Wheel(name) {}

Sedan_WheelRight::Sedan_WheelRight(const std::string& name) : Sedan_Wheel(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Sedan_Wheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(GetMeshFile(), false, false);
        trimesh->Transform(ChVector<>(0, m_offset, 0), ChMatrix33<>(1));
        m_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetStatic(true);
        m_trimesh_shape->SetName(GetMeshName());
        GetSpindle()->AddAsset(m_trimesh_shape);
    } else {
        ChWheel::AddVisualizationAssets(vis);
    }
}

void Sedan_Wheel::RemoveVisualizationAssets() {
    ChWheel::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by Sedan_Wheel::AddVisualizationAssets.
    // This is important for the ChWheel object because a tire may add its own assets
    // to the same body (the spindle).
    auto it = std::find(GetSpindle()->GetAssets().begin(), GetSpindle()->GetAssets().end(), m_trimesh_shape);
    if (it != GetSpindle()->GetAssets().end())
        GetSpindle()->GetAssets().erase(it);
}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
