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
// Authors: Rainer Gericke
// =============================================================================
//
// Marder road wheel subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/marder/Marder_SupportRoller.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace marder {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Marder_SupportRoller::m_wheel_mass = 10.56;
const ChVector<> Marder_SupportRoller::m_wheel_inertia(0.14, 1.16, 0.14);
const double Marder_SupportRoller::m_wheel_radius = 0.118;
const double Marder_SupportRoller::m_wheel_width = 0.181;
const double Marder_SupportRoller::m_wheel_gap = 0.051;

const std::string Marder_SupportRollerLeft::m_meshFile = "Marder/SupportRoller_L.obj";
const std::string Marder_SupportRollerRight::m_meshFile = "Marder/SupportRoller_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Marder_SupportRoller::Marder_SupportRoller(const std::string& name) : ChDoubleTrackWheel(name) {}

void Marder_SupportRoller::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.4f;
    minfo.cr = 0.75f;
    minfo.Y = 1e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Marder_SupportRoller::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(GetMeshFile(), false, false);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(GetMeshFile()).stem());
        trimesh_shape->SetMutable(false);
        m_wheel->AddVisualShape(trimesh_shape);
    } else {
        ChDoubleTrackWheel::AddVisualizationAssets(vis);
    }
}

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono
