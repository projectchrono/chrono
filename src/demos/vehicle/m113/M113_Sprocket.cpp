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

#include "m113/M113_Sprocket.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const int M113_Sprocket::m_num_teeth = 10;

const double M113_Sprocket::m_gear_mass = 436.7;
const ChVector<> M113_Sprocket::m_gear_inertia(12.22, 13.87, 12.22);
const double M113_Sprocket::m_axle_inertia = 1;
const double M113_Sprocket::m_separation = 0.225;

const double M113_Sprocket::m_gear_RT = 0.2605;
const double M113_Sprocket::m_gear_RC = 0.3;
const double M113_Sprocket::m_gear_R = 0.089;
const double M113_Sprocket::m_gear_RA = 0.245;

const std::string M113_SprocketLeft::m_meshName = "Sprocket_L_POV_geom";
const std::string M113_SprocketLeft::m_meshFile = vehicle::GetDataFile("M113/Sprocket_L.obj");

const std::string M113_SprocketRight::m_meshName = "Sprocket_R_POV_geom";
const std::string M113_SprocketRight::m_meshFile = vehicle::GetDataFile("M113/Sprocket_R.obj");

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_Sprocket::M113_Sprocket(const std::string& name)
    : ChArcSprocket(name), m_vis_type(PRIMITIVES) {
    SetContactMaterial(0.4f, 0.1f, 1e7f, 0.3f);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_Sprocket::AddGearVisualization() {
    switch (m_vis_type) {
        case PRIMITIVES:
            ChSprocket::AddGearVisualization();
            break;
        case MESH: {
            geometry::ChTriangleMeshConnected trimesh;
            trimesh.LoadWavefrontMesh(GetMeshFile(), false, false);
            auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(GetMeshName());
            m_gear->AddAsset(trimesh_shape);
            break;
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_Sprocket::ExportMeshPovray(const std::string& out_dir) {
    utils::WriteMeshPovray(GetMeshFile(), GetMeshName(), out_dir, ChColor(0.15f, 0.15f, 0.15f));
}

}  // end namespace m113
