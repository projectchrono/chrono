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

#include "chrono_thirdparty/filesystem/path.h"

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

const std::string M113a_TrackShoeSinglePin::m_meshFile = "M113/TrackShoe.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113a_TrackShoeSinglePin::M113a_TrackShoeSinglePin(const std::string& name) : ChTrackShoeSinglePin(name) {
    // Collision box: pad bottom (ground contact)
    BoxShape box_bottom(ChVector<>(0, 0, -0.015), QUNIT, ChVector<>(0.11, 0.19, 0.03), 0);

    // Collision box: pad top (wheel contact)
    BoxShape box_top(ChVector<>(0, 0, +0.015), QUNIT, ChVector<>(0.12, 0.20, 0.03), 1);

    // Collision box: guide pin (wheel contact)
    BoxShape box_guide(ChVector<>(0.045, 0, 0.0375), QUNIT, ChVector<>(0.0284, 0.0114, 0.075), 2);

    // Collision box: pad side outer (ground contact)
    BoxShape box_side_outer(ChVector<>(0, +0.16245, 0), QUNIT, ChVector<>(0.1315, 0.0542, 0.02), 0);

    // Collision box: pad side inner (ground contact)
    BoxShape box_side_inner(ChVector<>(0, -0.16245, 0), QUNIT, ChVector<>(0.1315, 0.0542, 0.02), 0);

    m_coll_boxes.push_back(box_bottom);
    m_coll_boxes.push_back(box_top);
    m_coll_boxes.push_back(box_guide);
    m_coll_boxes.push_back(box_side_outer);
    m_coll_boxes.push_back(box_side_inner);

    m_vis_boxes.push_back(box_bottom);
    m_vis_boxes.push_back(box_top);
    m_vis_boxes.push_back(box_guide);
    m_vis_boxes.push_back(box_side_outer);
    m_vis_boxes.push_back(box_side_inner);

    // Visualization cylinder: pin revolute joint
    m_vis_cylinders.push_back(CylinderShape(ChVector<>(0.077, 0, 0), QUNIT, 0.01, 0.399, -1));

    // Visualization cylinders: sprocket contact surfaces
    m_vis_cylinders.push_back(CylinderShape(ChVector<>(0.0535, -0.095, 0), QUNIT, 0.015, 0.095, -1));
    m_vis_cylinders.push_back(CylinderShape(ChVector<>(0.0535, +0.095, 0), QUNIT, 0.015, 0.095, -1));
    m_vis_cylinders.push_back(CylinderShape(ChVector<>(-0.061, -0.095, 0), QUNIT, 0.015, 0.095, -1));
    m_vis_cylinders.push_back(CylinderShape(ChVector<>(-0.061, +0.095, 0), QUNIT, 0.015, 0.095, -1));
}

void M113a_TrackShoeSinglePin::CreateContactMaterials(ChContactMethod contact_method) {
    // Material for cylindrical surface (sprocket contact)
    {
        MaterialInfo minfo;
        minfo.mu = 0.8f;
        minfo.cr = 0.1f;
        minfo.Y = 1e7f;
        m_cyl_material = minfo.CreateMaterial(contact_method);
    }

    // Material 0: pad bottom (ground contact)
    {
        MaterialInfo minfo;
        minfo.mu = 0.8f;
        minfo.cr = 0.1f;
        minfo.Y = 1e7f;
        m_shoe_materials.push_back(minfo.CreateMaterial(contact_method));
    }

    // Material 1: pad top (wheel contact)
    {
        MaterialInfo minfo;
        minfo.mu = 0.8f;
        minfo.cr = 0.1f;
        minfo.Y = 1e7f;
        m_shoe_materials.push_back(minfo.CreateMaterial(contact_method));
    }

    // Material 2: guide pin (wheel contact)
    {
        MaterialInfo minfo;
        minfo.mu = 0.8f;
        minfo.cr = 0.1f;
        minfo.Y = 1e7f;
        m_shoe_materials.push_back(minfo.CreateMaterial(contact_method));
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113a_TrackShoeSinglePin::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(m_meshFile).stem());
        trimesh_shape->SetStatic(true);
        m_shoe->AddAsset(trimesh_shape);
    } else {
        ChTrackShoeSinglePin::AddVisualizationAssets(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
