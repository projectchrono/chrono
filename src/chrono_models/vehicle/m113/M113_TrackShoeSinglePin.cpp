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
// M113 track shoe subsystem (single pin).
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/M113_TrackShoeSinglePin.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_TrackShoeSinglePin::m_shoe_height = 0.06;
const double M113_TrackShoeSinglePin::m_shoe_pitch = 0.154;
const double M113_TrackShoeSinglePin::m_shoe_mass = 18.02;
const ChVector<> M113_TrackShoeSinglePin::m_shoe_inertia(0.22, 0.04, 0.25);

const double M113_TrackShoeSinglePin::m_cyl_radius = 0.015;
const double M113_TrackShoeSinglePin::m_front_cyl_loc = 0.0535;
const double M113_TrackShoeSinglePin::m_rear_cyl_loc = -0.061;

const ChVector<> M113_TrackShoeSinglePin::m_pad_box_dims(0.11, 0.19, 0.06);
const ChVector<> M113_TrackShoeSinglePin::m_pad_box_loc(0, 0, 0);
const ChVector<> M113_TrackShoeSinglePin::m_guide_box_dims(0.0284, 0.0114, 0.075);
const ChVector<> M113_TrackShoeSinglePin::m_guide_box_loc(0.045, 0, 0.0375);

const std::string M113_TrackShoeSinglePin::m_meshName = "TrackShoe_POV_geom";
const std::string M113_TrackShoeSinglePin::m_meshFile = "M113/TrackShoe.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_TrackShoeSinglePin::M113_TrackShoeSinglePin(const std::string& name) : ChTrackShoeSinglePin(name) {
    SetContactFrictionCoefficient(0.8f);
    SetContactRestitutionCoefficient(0.1f);
    SetContactMaterialProperties(1e7f, 0.3f);
    SetContactMaterialCoefficients(2e5f, 40.0f, 2e5f, 20.0f);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_TrackShoeSinglePin::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(m_meshName);
        m_shoe->AddAsset(trimesh_shape);
    } else {
        ChTrackShoeSinglePin::AddVisualizationAssets(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
