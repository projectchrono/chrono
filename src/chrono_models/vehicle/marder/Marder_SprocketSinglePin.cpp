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
// Marder sprocket subsystem (single pin).
//
// =============================================================================

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/marder/Marder_SprocketSinglePin.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace marder {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const int Marder_SprocketSinglePin::m_num_teeth = 12;

const double Marder_SprocketSinglePin::m_gear_mass = 27.68;
const ChVector<> Marder_SprocketSinglePin::m_gear_inertia(0.646, 0.883, 0.646);
const double Marder_SprocketSinglePin::m_axle_inertia = 0.4;
const double Marder_SprocketSinglePin::m_separation = 0.225;

const double Marder_SprocketSinglePin::m_gear_RT = 0.3;          // Outer radius
const double Marder_SprocketSinglePin::m_gear_RC = 0.328414781;  // Arc centers radius
const double Marder_SprocketSinglePin::m_gear_R = 0.07;          // Arc radius
const double Marder_SprocketSinglePin::m_gear_RA = 0.275;        // Assembly radius

const double Marder_SprocketSinglePin::m_lateral_backlash = 0.01;

const std::string Marder_SprocketSinglePinLeft::m_meshFile = "Marder/Sprocket_L.obj";
const std::string Marder_SprocketSinglePinRight::m_meshFile = "Marder/Sprocket_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Marder_SprocketSinglePin::Marder_SprocketSinglePin(const std::string& name) : ChSprocketSinglePin(name) {}

void Marder_SprocketSinglePin::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.4f;
    minfo.cr = 0.75f;
    minfo.Y = 1e8f;
    m_material = minfo.CreateMaterial(contact_method);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Marder_SprocketSinglePin::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        ////auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(GetMeshFile(), false, false);
        auto trimesh = CreateVisualizationMesh(0.15, 0.03, 0.02);
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(GetMeshFile()).stem());
        trimesh_shape->SetMutable(false);
        m_gear->AddVisualShape(trimesh_shape);
    } else {
        ChSprocket::AddVisualizationAssets(vis);
    }
}

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono
