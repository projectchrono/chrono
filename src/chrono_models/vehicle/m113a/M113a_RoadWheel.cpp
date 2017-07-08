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
// M113 road wheel subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113a/M113a_RoadWheel.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113a_RoadWheel::m_wheel_mass = 56.05;
const ChVector<> M113a_RoadWheel::m_wheel_inertia(1.498, 2.607, 1.498);
const double M113a_RoadWheel::m_wheel_radius = 0.305;
const double M113a_RoadWheel::m_wheel_width = 0.18542;
const double M113a_RoadWheel::m_wheel_gap = 0.033;

const std::string M113a_RoadWheelLeft::m_meshName = "Roller_L_POV_geom";
const std::string M113a_RoadWheelLeft::m_meshFile = "M113/Roller_L.obj";

const std::string M113a_RoadWheelRight::m_meshName = "Roller_R_POV_geom";
const std::string M113a_RoadWheelRight::m_meshFile = "M113/Roller_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113a_RoadWheel::M113a_RoadWheel(const std::string& name) : ChDoubleRoadWheel(name) {
    SetContactFrictionCoefficient(0.7f);
    SetContactRestitutionCoefficient(0.1f);
    SetContactMaterialProperties(1e7f, 0.3f);
    SetContactMaterialCoefficients(2e5f, 40.0f, 2e5f, 20.0f);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113a_RoadWheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(GetMeshFile(), false, false);
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(GetMeshName());
        m_wheel->AddAsset(trimesh_shape);
    } else {
        ChDoubleRoadWheel::AddVisualizationAssets(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
