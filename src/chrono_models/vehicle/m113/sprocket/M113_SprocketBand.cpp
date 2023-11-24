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
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/sprocket/M113_SprocketBand.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_SprocketBand::m_gear_mass = 27.68;
const ChVector<> M113_SprocketBand::m_gear_inertia(0.646, 0.883, 0.646);
const double M113_SprocketBand::m_axle_inertia = 0.4;
const double M113_SprocketBand::m_separation = 0.278;

// Gear profile data
const int M113_SprocketBand::m_num_teeth = 17;
const double M113_SprocketBand::m_gear_outer_radius = 0.2307 * 1.04;
const double M113_SprocketBand::m_gear_base_width = 0.0530 * 1.04;
const double M113_SprocketBand::m_gear_tip_width = 0.0128 * 1.04;
const double M113_SprocketBand::m_gear_tooth_depth = 0.0387 * 1.04;
const double M113_SprocketBand::m_gear_arc_radius = 0.0542 * 1.04;
const double M113_SprocketBand::m_gear_RA = 0.2307 * 1.04;
const double M113_SprocketBand::m_gear_RT = 0.22;

const double M113_SprocketBand::m_lateral_backlash = 0.02;

const std::string M113_SprocketBandLeft::m_meshFile = "M113/meshes/SprocketBand_L.obj";
const std::string M113_SprocketBandRight::m_meshFile = "M113/meshes/SprocketBand_R.obj";

// -----------------------------------------------------------------------------

M113_SprocketBand::M113_SprocketBand(const std::string& name) : ChSprocketBand(name) {}

void M113_SprocketBand::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.4f;
    minfo.cr = 0.75f;
    minfo.Y = 1e9f;
    m_material = minfo.CreateMaterial(contact_method);
}

void M113_SprocketBand::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(GetMeshFile(), false, false);
        ////auto trimesh = CreateVisualizationMesh(0.15, 0.03, 0.02);
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(GetMeshFile()).stem());
        trimesh_shape->SetMutable(false);
        m_gear->AddVisualShape(trimesh_shape);
    } else {
        ChSprocketBand::AddVisualizationAssets(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
