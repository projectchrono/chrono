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
// M113 sprocket subsystem (single pin).
//
// =============================================================================

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/M113_SprocketSinglePin.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const int M113_SprocketSinglePin::m_num_teeth = 10;

const double M113_SprocketSinglePin::m_gear_mass = 27.68;
const ChVector<> M113_SprocketSinglePin::m_gear_inertia(0.646, 0.883, 0.646);
const double M113_SprocketSinglePin::m_axle_inertia = 0.02;
const double M113_SprocketSinglePin::m_separation = 0.225;

const double M113_SprocketSinglePin::m_gear_RT = 0.2605;
const double M113_SprocketSinglePin::m_gear_RC = 0.3;
const double M113_SprocketSinglePin::m_gear_R = 0.089;
const double M113_SprocketSinglePin::m_gear_RA = 0.245;

const std::string M113_SprocketSinglePinLeft::m_meshName = "Sprocket_L_POV_geom";
const std::string M113_SprocketSinglePinLeft::m_meshFile = "M113/Sprocket_L.obj";

const std::string M113_SprocketSinglePinRight::m_meshName = "Sprocket_R_POV_geom";
const std::string M113_SprocketSinglePinRight::m_meshFile = "M113/Sprocket_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_SprocketSinglePin::M113_SprocketSinglePin(const std::string& name) : ChSprocketSinglePin(name) {
    SetContactFrictionCoefficient(0.4f);
    SetContactRestitutionCoefficient(0.1f);
    SetContactMaterialProperties(1e7f, 0.3f);
    SetContactMaterialCoefficients(2e5f, 40.0f, 2e5f, 20.0f);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_SprocketSinglePin::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(GetMeshFile(), false, false);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(GetMeshName());
        trimesh_shape->SetStatic(true);
        m_gear->AddAsset(trimesh_shape);
    } else {
        ChSprocket::AddVisualizationAssets(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
