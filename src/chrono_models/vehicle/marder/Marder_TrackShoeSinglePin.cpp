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
const ChVector3d Marder_TrackShoeSinglePin::m_shoe_inertia(0.22, 0.04, 0.25);

const double Marder_TrackShoeSinglePin::m_cyl_radius = 0.015;
const double Marder_TrackShoeSinglePin::m_front_cyl_loc = 0.0535;
const double Marder_TrackShoeSinglePin::m_rear_cyl_loc = -0.061;

const ChVector3d Marder_TrackShoeSinglePin::m_pin_center(0.045, 0, 0.0375);

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
        m_geometry.materials.push_back(minfo);
    }

    // Material 1: pad top (wheel contact)
    {
        ChContactMaterialData minfo;
        minfo.mu = 0.8f;
        minfo.cr = 0.75f;
        minfo.Y = 1e7f;
        m_geometry.materials.push_back(minfo);
    }

    // Material 2: guide pin (wheel contact)
    {
        ChContactMaterialData minfo;
        minfo.mu = 0.8f;
        minfo.cr = 0.75f;
        minfo.Y = 1e7f;
        m_geometry.materials.push_back(minfo);
    }

    // Geometry

    // Collision box: pad bottom (ground contact)
    utils::ChBodyGeometry::BoxShape box_bottom(ChVector3d(0, 0, -0.015), QUNIT, ChVector3d(0.11, 0.4, 0.03), 0);

    // Collision box: pad top (wheel contact)
    utils::ChBodyGeometry::BoxShape box_top(ChVector3d(0, 0, +0.015), QUNIT, ChVector3d(0.10, 0.4, 0.03), 1);

    // Collision box: guide pin (wheel contact)
    utils::ChBodyGeometry::BoxShape box_guide(ChVector3d(0.045, 0, 0.0375), QUNIT, ChVector3d(0.0284, 0.0114, 0.075),
                                              2);

    // Collision box: pad side outer (ground contact)
    utils::ChBodyGeometry::BoxShape box_side_outer(ChVector3d(0, +0.43 / 2, 0), QUNIT, ChVector3d(0.1315, 0.0542, 0.02),
                                                   0);

    // Collision box: pad side inner (ground contact)
    utils::ChBodyGeometry::BoxShape box_side_inner(ChVector3d(0, -0.43 / 2, 0), QUNIT, ChVector3d(0.1315, 0.0542, 0.02),
                                                   0);

    m_geometry.coll_boxes.push_back(box_bottom);
    m_geometry.coll_boxes.push_back(box_top);
    m_geometry.coll_boxes.push_back(box_guide);
    m_geometry.coll_boxes.push_back(box_side_outer);
    m_geometry.coll_boxes.push_back(box_side_inner);

    m_ground_geometry.materials = m_geometry.materials;
    m_ground_geometry.coll_boxes.push_back(box_bottom);
    m_ground_geometry.coll_boxes.push_back(box_side_outer);
    m_ground_geometry.coll_boxes.push_back(box_side_inner);

    m_geometry.vis_boxes.push_back(box_bottom);
    m_geometry.vis_boxes.push_back(box_top);
    m_geometry.vis_boxes.push_back(box_guide);
    m_geometry.vis_boxes.push_back(box_side_outer);
    m_geometry.vis_boxes.push_back(box_side_inner);

    // Visualization cylinder: pin revolute joint
    m_geometry.vis_cylinders.push_back(
        utils::ChBodyGeometry::CylinderShape(ChVector3d(0.077, 0, 0), ChVector3d(0, 1, 0), 0.01, 0.399, -1));

    // Visualization cylinders: sprocket contact surfaces
    m_geometry.vis_cylinders.push_back(
        utils::ChBodyGeometry::CylinderShape(ChVector3d(0.0535, -0.095, 0), ChVector3d(0, 1, 0), 0.015, 0.095, -1));
    m_geometry.vis_cylinders.push_back(
        utils::ChBodyGeometry::CylinderShape(ChVector3d(0.0535, +0.095, 0), ChVector3d(0, 1, 0), 0.015, 0.095, -1));
    m_geometry.vis_cylinders.push_back(
        utils::ChBodyGeometry::CylinderShape(ChVector3d(-0.061, -0.095, 0), ChVector3d(0, 1, 0), 0.015, 0.095, -1));
    m_geometry.vis_cylinders.push_back(
        utils::ChBodyGeometry::CylinderShape(ChVector3d(-0.061, +0.095, 0), ChVector3d(0, 1, 0), 0.015, 0.095, -1));

    m_geometry.vis_mesh_file = vehicle::GetDataFile("vehicle/M113/TrackShoe.obj");
}

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono
