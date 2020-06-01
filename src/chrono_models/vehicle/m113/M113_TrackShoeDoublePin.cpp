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

#include "chrono_models/vehicle/m113/M113_TrackShoeDoublePin.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double M113_TrackShoeDoublePin::m_shoe_mass = 18.02;
const ChVector<> M113_TrackShoeDoublePin::m_shoe_inertia(0.22, 0.04, 0.25);
const double M113_TrackShoeDoublePin::m_shoe_length = 0.0984;  // 3.875''
const double M113_TrackShoeDoublePin::m_shoe_width = 0.2781;   // 10.95''
const double M113_TrackShoeDoublePin::m_shoe_height = 0.06;

const ChVector<> M113_TrackShoeDoublePin::m_pin_center(0.045, 0, 0.0375);

const double M113_TrackShoeDoublePin::m_connector_mass = 2.0;                  //// TODO
const ChVector<> M113_TrackShoeDoublePin::m_connector_inertia(0.1, 0.1, 0.1);  //// TODO
const double M113_TrackShoeDoublePin::m_connector_radius = 0.02;               // 0.88''
const double M113_TrackShoeDoublePin::m_connector_length = 0.054;              // 2.125''
const double M113_TrackShoeDoublePin::m_connector_width = 0.02;

const std::string M113_TrackShoeDoublePin::m_meshFile = "M113/TrackShoeDoublePin.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_TrackShoeDoublePin::M113_TrackShoeDoublePin(const std::string& name) : ChTrackShoeDoublePin(name) {
    // Collision box: pad bottom (ground contact)
    BoxShape box_bottom(ChVector<>(0, 0, -0.015), QUNIT, ChVector<>(0.11, 0.19, 0.03), 0);

    // Collision box: pad top (wheel contact)
    BoxShape box_top(ChVector<>(0, 0, +0.015), QUNIT, ChVector<>(0.10, 0.18, 0.03), 1);

    // Collision box: guide pin (wheel contact)
    BoxShape box_guide(ChVector<>(0.045, 0, 0.0375), QUNIT, ChVector<>(0.0284, 0.0114, 0.075), 2);

    m_coll_boxes.push_back(box_bottom);
    m_coll_boxes.push_back(box_top);
    m_coll_boxes.push_back(box_guide);

    m_vis_boxes.push_back(box_bottom);
    m_vis_boxes.push_back(box_top);
    m_vis_boxes.push_back(box_guide);

    // Visualization cylinder: pin revolute joints
    m_vis_cylinders.push_back(CylinderShape(ChVector<>(+0.0492, 0, 0), QUNIT, 0.01, 0.3381, -1));
    m_vis_cylinders.push_back(CylinderShape(ChVector<>(-0.0492, 0, 0), QUNIT, 0.01, 0.3381, -1));
}

void M113_TrackShoeDoublePin::CreateContactMaterials(ChContactMethod contact_method) {
    // Material for connectors (sprocket contact)
    {
        MaterialInfo minfo;
        minfo.mu = 0.8f;
        minfo.cr = 0.1f;
        minfo.Y = 1e7f;
        m_conn_material = minfo.CreateMaterial(contact_method);
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
void M113_TrackShoeDoublePin::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        //// TODO:
        //// Set up meshes for shoe and connectors
        //// For now, default to PRIMITIVE visualization
        ChTrackShoeDoublePin::AddVisualizationAssets(vis);
    } else {
        ChTrackShoeDoublePin::AddVisualizationAssets(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
