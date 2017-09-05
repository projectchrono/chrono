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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// M113 sprocket subsystem (continuous band track).
//
// =============================================================================

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/M113_SprocketCB.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_SprocketCB::m_gear_mass = 436.7;
const ChVector<> M113_SprocketCB::m_gear_inertia(12.22, 13.87, 12.22);
const double M113_SprocketCB::m_axle_inertia = 1;
const double M113_SprocketCB::m_separation = 0.278;

// Gear profile data
const int M113_SprocketCB::m_num_teeth = 17;
const double M113_SprocketCB::m_gear_outer_radius = 0.2307;
const double M113_SprocketCB::m_gear_base_width = 0.0530;
const double M113_SprocketCB::m_gear_tip_width = 0.0128;
const double M113_SprocketCB::m_gear_tooth_depth = 0.0387;
const double M113_SprocketCB::m_gear_arc_radius = 0.0542;
const double M113_SprocketCB::m_gear_RA = 0.2307;

const std::string M113_SprocketCBLeft::m_meshName = "Sprocket2_L_POV_geom";
const std::string M113_SprocketCBLeft::m_meshFile = "M113/Sprocket2_L.obj";

const std::string M113_SprocketCBRight::m_meshName = "Sprocket2_R_POV_geom";
const std::string M113_SprocketCBRight::m_meshFile = "M113/Sprocket2_R.obj";



// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_SprocketCB::M113_SprocketCB(const std::string& name) : ChSprocketCB(name) {
    SetContactFrictionCoefficient(0.4f);
    SetContactRestitutionCoefficient(0.1f);
    SetContactMaterialProperties(1e7f, 0.3f);
    SetContactMaterialCoefficients(2e5f, 40.0f, 2e5f, 20.0f);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_SprocketCB::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        //// TODO
        //// Set up mesh for sprocket gear
        //// For now, default to rendering the profile.
        ChSprocket::AddVisualizationAssets(vis);
        ////geometry::ChTriangleMeshConnected trimesh;
        ////trimesh.LoadWavefrontMesh(GetMeshFile(), false, false);
        ////auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        ////trimesh_shape->SetMesh(trimesh);
        ////trimesh_shape->SetName(GetMeshName());
        ////m_gear->AddAsset(trimesh_shape);
    } else {
        ChSprocket::AddVisualizationAssets(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
