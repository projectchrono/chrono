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

#include "m113/M113_RoadWheel.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_RoadWheel::m_wheel_mass = 1;
const ChVector<> M113_RoadWheel::m_wheel_inertia(1, 1, 1);
const double M113_RoadWheel::m_wheel_radius = 0.25;
const double M113_RoadWheel::m_wheel_width = 0.2;
const double M113_RoadWheel::m_wheel_gap = 0.06;

const std::string M113_RoadWheel::m_meshName = "roller_POV_geom";
const std::string M113_RoadWheel::m_meshFile = vehicle::GetDataFile("M113/Roller_XforwardYup.obj");

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_RoadWheel::M113_RoadWheel(VisualizationType vis_type) : ChDoubleRoadWheel("M113_RoadWheel"), m_vis_type(vis_type) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_RoadWheel::Initialize(ChSharedPtr<ChBodyAuxRef> chassis, const ChVector<>& location) {
    // Invoke the base class method to perform the actual initialization
    ChDoubleRoadWheel::Initialize(chassis, location);

    // Attach visualization
    switch (m_vis_type) {
        case PRIMITIVES:
            AddWheelVisualization();

            break;
        case MESH: {
            geometry::ChTriangleMeshConnected trimesh;
            trimesh.LoadWavefrontMesh(m_meshFile, false, false);

            ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(m_meshName);
            m_wheel->AddAsset(trimesh_shape);

            break;
        }
    }
}

}  // end namespace m113
