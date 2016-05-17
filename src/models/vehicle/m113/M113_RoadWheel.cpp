// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#include "models/vehicle/m113/M113_RoadWheel.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_RoadWheel::m_wheel_mass = 561.1;
const ChVector<> M113_RoadWheel::m_wheel_inertia(19.82, 26.06, 19.82);
const double M113_RoadWheel::m_wheel_radius = 0.305;
const double M113_RoadWheel::m_wheel_width = 0.181;
const double M113_RoadWheel::m_wheel_gap = 0.051;

const std::string M113_RoadWheelLeft::m_meshName = "Roller_L_POV_geom";
const std::string M113_RoadWheelLeft::m_meshFile = vehicle::GetDataFile("M113/Roller_L.obj");

const std::string M113_RoadWheelRight::m_meshName = "Roller_R_POV_geom";
const std::string M113_RoadWheelRight::m_meshFile = vehicle::GetDataFile("M113/Roller_R.obj");

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_RoadWheel::M113_RoadWheel(const std::string& name)
    : ChDoubleRoadWheel(name), m_vis_type(PRIMITIVES) {
    SetContactMaterial(0.7f, 0.1f, 1e7f, 0.3f);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_RoadWheel::AddWheelVisualization() {
    switch (m_vis_type) {
        case PRIMITIVES:
            ChDoubleRoadWheel::AddWheelVisualization();
            break;
        case MESH: {
            geometry::ChTriangleMeshConnected trimesh;
            trimesh.LoadWavefrontMesh(GetMeshFile(), false, false);
            auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(GetMeshName());
            m_wheel->AddAsset(trimesh_shape);
            break;
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_RoadWheel::ExportMeshPovray(const std::string& out_dir) {
    utils::WriteMeshPovray(GetMeshFile(), GetMeshName(), out_dir, ChColor(0.15f, 0.15f, 0.15f));
}

}  // end namespace m113
