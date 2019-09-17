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

#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113a/M113a_TrackShoeSinglePin.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113a_TrackShoeSinglePin::m_shoe_height = 0.06;
const double M113a_TrackShoeSinglePin::m_shoe_pitch = 0.1524;
const double M113a_TrackShoeSinglePin::m_shoe_mass = 8.703;
const ChVector<> M113a_TrackShoeSinglePin::m_shoe_inertia(0.153, 0.065, 0.215);

const double M113a_TrackShoeSinglePin::m_cyl_radius = 0.015;
const double M113a_TrackShoeSinglePin::m_front_cyl_loc = 0.0535;
const double M113a_TrackShoeSinglePin::m_rear_cyl_loc = -0.061;

const ChVector<> M113a_TrackShoeSinglePin::m_pad_box_dims(0.11, 0.19, 0.06);
const ChVector<> M113a_TrackShoeSinglePin::m_pad_box_loc(0, 0, 0);
const ChVector<> M113a_TrackShoeSinglePin::m_guide_box_dims(0.0284, 0.0114, 0.075);
const ChVector<> M113a_TrackShoeSinglePin::m_guide_box_loc(0.045, 0, 0.0375);

const std::string M113a_TrackShoeSinglePin::m_meshName = "TrackShoe_POV_geom";
const std::string M113a_TrackShoeSinglePin::m_meshFile = "M113/TrackShoe.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113a_TrackShoeSinglePin::M113a_TrackShoeSinglePin(const std::string& name) : ChTrackShoeSinglePin(name) {
    SetContactFrictionCoefficient(0.8f);
    SetContactRestitutionCoefficient(0.1f);
    SetContactMaterialProperties(1e7f, 0.3f);
    SetContactMaterialCoefficients(2e5f, 40.0f, 2e5f, 20.0f);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113a_TrackShoeSinglePin::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(m_meshName);
        trimesh_shape->SetStatic(true);
        m_shoe->AddAsset(trimesh_shape);
    } else {
        ChTrackShoeSinglePin::AddVisualizationAssets(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
