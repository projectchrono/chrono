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
// MAN Kat 1 wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/man/MAN_5t_Wheel.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double MAN_5t_Wheel::m_mass = 30.0;
const ChVector<> MAN_5t_Wheel::m_inertia(.6, .63, .6);

const double MAN_5t_Wheel::m_radius = 0.254;
const double MAN_5t_Wheel::m_width = 0.254;

const std::string MAN_5t_Wheel::m_meshFile = "MAN_Kat1/meshes/MAN_rim.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MAN_5t_Wheel::MAN_5t_Wheel(const std::string& name) : ChWheel(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void MAN_5t_Wheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        ChQuaternion<> rot = (m_side == VehicleSide::LEFT) ? Q_from_AngZ(0) : Q_from_AngZ(CH_C_PI);
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(GetDataFile(m_meshFile), false, false);
        trimesh->Transform(ChVector<>(0, m_offset, 0), ChMatrix33<>(rot));
        m_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->Pos = ChVector<>(0, m_offset, 0);
        m_trimesh_shape->Rot = ChMatrix33<>(rot);
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetStatic(true);
        m_trimesh_shape->SetName(filesystem::path(m_meshFile).stem());
        GetSpindle()->AddAsset(m_trimesh_shape);
    } else {
        ChWheel::AddVisualizationAssets(vis);
    }
}

void MAN_5t_Wheel::RemoveVisualizationAssets() {
    ChWheel::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by MAN_5t_Wheel::AddVisualizationAssets.
    // This is important for the ChWheel object because a tire may add its own assets
    // to the same body (the spindle).
    auto it = std::find(GetSpindle()->GetAssets().begin(), GetSpindle()->GetAssets().end(), m_trimesh_shape);
    if (it != GetSpindle()->GetAssets().end())
        GetSpindle()->GetAssets().erase(it);
}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
