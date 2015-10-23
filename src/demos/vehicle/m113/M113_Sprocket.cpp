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
// M113 sprocket subsystem.
//
// =============================================================================

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChSprocketProfiles.h"

#include "m113/M113_Sprocket.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const int M113_Sprocket::m_num_teeth = 10;

const double M113_Sprocket::m_gear_mass = 1;
const ChVector<> M113_Sprocket::m_gear_inertia(1, 1, 1);
const double M113_Sprocket::m_axle_inertia = 1;
const double M113_Sprocket::m_separation = 0.2;

const double M113_Sprocket::m_gear_RT = 0.2605;
const double M113_Sprocket::m_gear_RC = 0.3;
const double M113_Sprocket::m_gear_R = 0.089;

const std::string M113_Sprocket::m_meshName = "sprocket_POV_geom";
const std::string M113_Sprocket::m_meshFile = vehicle::GetDataFile("M113/Sprocket_XforwardYup.obj");

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_Sprocket::M113_Sprocket(VisualizationType vis_type) : ChSprocket("M113_Sprocket"), m_vis_type(vis_type) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSharedPtr<geometry::ChLinePath> M113_Sprocket::getProfile() {
    return ChCircularProfile(m_num_teeth, m_gear_RT, m_gear_RC, m_gear_R);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_Sprocket::Initialize(ChSharedPtr<ChBodyAuxRef> chassis, const ChVector<>& location) {
    // Invoke the base class method to perform the actual initialization
    ChSprocket::Initialize(chassis, location);

    // Attach visualization
    switch (m_vis_type) {
        case PRIMITIVES:
            AddGearVisualization(ChColor(1, 0, 0));

            break;
        case MESH: {
            geometry::ChTriangleMeshConnected trimesh;
            trimesh.LoadWavefrontMesh(m_meshFile, false, false);

            ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(m_meshName);
            m_gear->AddAsset(trimesh_shape);

            break;
        }
    }
}

}  // end namespace m113
