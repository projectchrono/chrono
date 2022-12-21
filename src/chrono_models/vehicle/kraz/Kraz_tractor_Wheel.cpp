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

#include "chrono_models/vehicle/kraz/Kraz_tractor_Wheel.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace kraz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Kraz_tractor_Wheel::m_mass = 30.0;
const ChVector<> Kraz_tractor_Wheel::m_inertia(0.6, 0.63, 0.6);

const double Kraz_tractor_Wheel::m_radius = 0.254;
const double Kraz_tractor_Wheel::m_width = 0.254;

const std::string Kraz_tractor_Wheel::m_meshFile = "longhaul/meshes/SemiTractor_rim.obj";

Kraz_tractor_Wheel::Kraz_tractor_Wheel(const std::string& name) : ChWheel(name) {}

void Kraz_tractor_Wheel::AddVisualizationAssets(chrono::vehicle::VisualizationType vis) {
    if (vis == chrono::vehicle::VisualizationType::MESH) {
        chrono::ChQuaternion<> rot = (m_side == VehicleSide::LEFT) ? Q_from_AngZ(0) : Q_from_AngZ(CH_C_PI);
        auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(GetDataFile(m_meshFile), false, false);
        trimesh->Transform(ChVector<>(0, m_offset, 0), chrono::ChMatrix33<>(rot));
        m_trimesh_shape = chrono_types::make_shared<chrono::ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetMutable(false);
        m_trimesh_shape->SetName(filesystem::path(m_meshFile).stem());
        GetSpindle()->AddVisualShape(m_trimesh_shape, ChFrame<>(ChVector<>(0, m_offset, 0), rot));
    } else {
        ChWheel::AddVisualizationAssets(vis);
    }
}

void Kraz_tractor_Wheel::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(GetSpindle(), m_trimesh_shape);
    ChWheel::RemoveVisualizationAssets();
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
