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
// Marder track shoe subsystem (single pin).
//
// =============================================================================

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/marder/Marder_TrackShoeSinglePin.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace marder {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Marder_TrackShoeSinglePin::m_shoe_height = 0.06;
const double Marder_TrackShoeSinglePin::m_shoe_pitch = 0.17;
const double Marder_TrackShoeSinglePin::m_shoe_mass = 18.02;
const ChVector<> Marder_TrackShoeSinglePin::m_shoe_inertia(0.22, 0.04, 0.25);

const double Marder_TrackShoeSinglePin::m_cyl_radius = 0.015;
const double Marder_TrackShoeSinglePin::m_front_cyl_loc = 0.0535;
const double Marder_TrackShoeSinglePin::m_rear_cyl_loc = -0.061;

const ChVector<> Marder_TrackShoeSinglePin::m_pin_center(0.045, 0, 0.0375);

// -----------------------------------------------------------------------------

Marder_TrackShoeSinglePin::Marder_TrackShoeSinglePin(const std::string& name) : ChTrackShoeSinglePin(name) {
    // Contact materials

    // Material for cylindrical surfaces (sprocket contact)
    m_shoe_sprk_minfo.mu = 0.8f;
    m_shoe_sprk_minfo.cr = 0.75f;
    m_shoe_sprk_minfo.Y = 1e8f;

    // Material 0: pad bottom (ground contact)
    {
        ChContactMaterialData minfo;
        minfo.mu = 0.8f;
        minfo.cr = 0.75f;
        minfo.Y = 1e7f;
        m_geometry.m_materials.push_back(minfo);
    }

    // Material 1: pad top (wheel contact)
    {
        ChContactMaterialData minfo;
        minfo.mu = 0.8f;
        minfo.cr = 0.75f;
        minfo.Y = 1e7f;
        m_geometry.m_materials.push_back(minfo);
    }

    // Material 2: guide pin (wheel contact)
    {
        ChContactMaterialData minfo;
        minfo.mu = 0.8f;
        minfo.cr = 0.75f;
        minfo.Y = 1e7f;
        m_geometry.m_materials.push_back(minfo);
    }

    // Geometry
    
    // Collision box: pad bottom (ground contact)
    ChVehicleGeometry::BoxShape box_bottom(ChVector<>(0, 0, -0.015), QUNIT, ChVector<>(0.11, 0.4, 0.03), 0);

    // Collision box: pad top (wheel contact)
    ChVehicleGeometry::BoxShape box_top(ChVector<>(0, 0, +0.015), QUNIT, ChVector<>(0.10, 0.4, 0.03), 1);

    // Collision box: guide pin (wheel contact)
    ChVehicleGeometry::BoxShape box_guide(ChVector<>(0.045, 0, 0.0375), QUNIT, ChVector<>(0.0284, 0.0114, 0.075), 2);

    // Collision box: pad side outer (ground contact)
    ChVehicleGeometry::BoxShape box_side_outer(ChVector<>(0, +0.43 / 2, 0), QUNIT, ChVector<>(0.1315, 0.0542, 0.02), 0);

    // Collision box: pad side inner (ground contact)
    ChVehicleGeometry::BoxShape box_side_inner(ChVector<>(0, -0.43 / 2, 0), QUNIT, ChVector<>(0.1315, 0.0542, 0.02), 0);

    m_geometry.m_has_collision = true;
    m_geometry.m_coll_boxes.push_back(box_bottom);
    m_geometry.m_coll_boxes.push_back(box_top);
    m_geometry.m_coll_boxes.push_back(box_guide);
    m_geometry.m_coll_boxes.push_back(box_side_outer);
    m_geometry.m_coll_boxes.push_back(box_side_inner);

    m_ground_geometry.m_has_collision = true;
    m_ground_geometry.m_materials = m_geometry.m_materials;
    m_ground_geometry.m_coll_boxes.push_back(box_bottom);
    m_ground_geometry.m_coll_boxes.push_back(box_side_outer);
    m_ground_geometry.m_coll_boxes.push_back(box_side_inner);    

    m_geometry.m_has_primitives = true;
    m_geometry.m_vis_boxes.push_back(box_bottom);
    m_geometry.m_vis_boxes.push_back(box_top);
    m_geometry.m_vis_boxes.push_back(box_guide);
    m_geometry.m_vis_boxes.push_back(box_side_outer);
    m_geometry.m_vis_boxes.push_back(box_side_inner);

    // Visualization cylinder: pin revolute joint
    m_geometry.m_vis_cylinders.push_back(
        ChVehicleGeometry::CylinderShape(ChVector<>(0.077, 0, 0), ChVector<>(0, 1, 0), 0.01, 0.399, -1));

    // Visualization cylinders: sprocket contact surfaces
    m_geometry.m_vis_cylinders.push_back(
        ChVehicleGeometry::CylinderShape(ChVector<>(0.0535, -0.095, 0), ChVector<>(0, 1, 0), 0.015, 0.095, -1));
    m_geometry.m_vis_cylinders.push_back(
        ChVehicleGeometry::CylinderShape(ChVector<>(0.0535, +0.095, 0), ChVector<>(0, 1, 0), 0.015, 0.095, -1));
    m_geometry.m_vis_cylinders.push_back(
        ChVehicleGeometry::CylinderShape(ChVector<>(-0.061, -0.095, 0), ChVector<>(0, 1, 0), 0.015, 0.095, -1));
    m_geometry.m_vis_cylinders.push_back(
        ChVehicleGeometry::CylinderShape(ChVector<>(-0.061, +0.095, 0), ChVector<>(0, 1, 0), 0.015, 0.095, -1));

    m_geometry.m_has_mesh = true;
    m_geometry.m_vis_mesh_file = "M113/TrackShoe.obj";
}

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono
