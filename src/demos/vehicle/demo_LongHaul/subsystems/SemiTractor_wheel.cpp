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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// SemiTractor 28t wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "subsystems/SemiTractor_wheel.h"
#include "chrono_thirdparty/filesystem/path.h"

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double SemiTractor_wheel::m_mass = 30.0;
const chrono::ChVector<> SemiTractor_wheel::m_inertia(.6, .63, .6);

const double SemiTractor_wheel::m_radius = 0.254;
const double SemiTractor_wheel::m_width = 0.254;

const std::string SemiTractor_wheel::m_meshFile = "longhaul/meshes/SemiTractor_rim.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SemiTractor_wheel::SemiTractor_wheel(const std::string& name) : ChWheel(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SemiTractor_wheel::AddVisualizationAssets(chrono::vehicle::VisualizationType vis) {
    if (vis == chrono::vehicle::VisualizationType::MESH) {
        chrono::ChQuaternion<> rot = (m_side == chrono::vehicle::VehicleSide::LEFT)
                                         ? chrono::Q_from_AngZ(0)
                                         : chrono::Q_from_AngZ(chrono::CH_C_PI);
        auto trimesh = chrono_types::make_shared<chrono::geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(chrono::vehicle::GetDataFile(m_meshFile), false, false);
        trimesh->Transform(chrono::ChVector<>(0, m_offset, 0), chrono::ChMatrix33<>(rot));
        m_trimesh_shape = chrono_types::make_shared<chrono::ChTriangleMeshShape>();
        m_trimesh_shape->Pos = chrono::ChVector<>(0, m_offset, 0);
        m_trimesh_shape->Rot = chrono::ChMatrix33<>(rot);
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetStatic(true);
        m_trimesh_shape->SetName(filesystem::path(m_meshFile).stem());
        GetSpindle()->AddAsset(m_trimesh_shape);
    } else {
        ChWheel::AddVisualizationAssets(vis);
    }
}

void SemiTractor_wheel::RemoveVisualizationAssets() {
    ChWheel::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by SemiTractor_wheel::AddVisualizationAssets.
    // This is important for the ChWheel object because a tire may add its own assets
    // to the same body (the spindle).
    auto it = std::find(GetSpindle()->GetAssets().begin(), GetSpindle()->GetAssets().end(), m_trimesh_shape);
    if (it != GetSpindle()->GetAssets().end())
        GetSpindle()->GetAssets().erase(it);
}
