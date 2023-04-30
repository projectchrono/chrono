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
// M113 track shoe subsystem (continuous band with rigid links).
//
// =============================================================================

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/track_shoe/M113_TrackShoeBandBushing.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

//// TODO: check these values
//// Web Iyy is larger for better numerical scaling.  Actual value is closer to 4e-5 kg*m^2
const double M113_TrackShoeBandBushing::m_tread_mass = 1.8;
const double M113_TrackShoeBandBushing::m_web_mass = 0.33;
const ChVector<> M113_TrackShoeBandBushing::m_tread_inertias(0.015, 0.001, 0.016);
const ChVector<> M113_TrackShoeBandBushing::m_web_inertias(0.003, 0.001, 0.003);

const double M113_TrackShoeBandBushing::m_shoe_height = 0.06;

const double M113_TrackShoeBandBushing::m_belt_width = 0.3175;  // 12.5 in

const double M113_TrackShoeBandBushing::m_tooth_tip_length = 0.0126 * 1.04;
const double M113_TrackShoeBandBushing::m_tooth_base_length = 0.0529 * 1.04;
const double M113_TrackShoeBandBushing::m_tooth_width = 0.0508;  // 2 in
const double M113_TrackShoeBandBushing::m_tooth_height = 0.0385 * 1.04;
const double M113_TrackShoeBandBushing::m_tooth_arc_radius = 0.0540 * 1.04;

const int M113_TrackShoeBandBushing::m_num_web_segments = 1;
const double M113_TrackShoeBandBushing::m_web_length = 0.0335 * 1.04;
const double M113_TrackShoeBandBushing::m_web_thickness = 0.0188 * 1.04;

const double M113_TrackShoeBandBushing::m_tread_length = 0.0724 * 1.04;
const double M113_TrackShoeBandBushing::m_tread_thickness = 0.0157 * 1.04;

const ChVector<> M113_TrackShoeBandBushing::m_guide_box_dims(0.0529, 0.0114, 0.075);
const double M113_TrackShoeBandBushing::m_guide_box_offset_x = 0;

const std::string M113_TrackShoeBandBushing::m_meshFile = "M113/meshes/TrackShoeBand.obj";
const std::string M113_TrackShoeBandBushing::m_tread_meshName = "M113_Tread";

// -----------------------------------------------------------------------------
M113_TrackShoeBandBushing::M113_TrackShoeBandBushing(const std::string& name) : ChTrackShoeBandBushing(name) {
    m_bushingData = chrono_types::make_shared<ChVehicleBushingData>();
    m_bushingData->K_lin = 7e7;
    m_bushingData->D_lin = 0.05 * 7e7;
    m_bushingData->K_rot = 1e5;
    m_bushingData->D_rot= 0.05 * 1e5;
    m_bushingData->K_lin_dof = 0;
    m_bushingData->D_lin_dof = 0;
    m_bushingData->K_rot_dof = 500;
    m_bushingData->D_rot_dof = 0.05 * 500;

    // Pad material (ground contact)
    m_pad_matinfo.mu = 0.8f;
    m_pad_matinfo.cr = 0.75f;
    m_pad_matinfo.Y = 1e7f;

    // Body material (wheel contact)
    m_body_matinfo.mu = 0.8f;
    m_body_matinfo.cr = 0.75f;
    m_body_matinfo.Y = 1e7f;

    // Guide material (wheel contact)
    m_guide_matinfo = m_body_matinfo;

    // Tooth material (sprocket contact)
    m_tooth_matinfo.mu = 0.8f;
    m_tooth_matinfo.cr = 0.75f;
    m_tooth_matinfo.Y = 1e9f;
}

// -----------------------------------------------------------------------------
void M113_TrackShoeBandBushing::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(m_meshFile, false, false);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(m_meshFile).stem());
        trimesh_shape->SetMutable(false);
        m_shoe->AddVisualShape(trimesh_shape);
    } else {
        ChTrackShoeBandBushing::AddVisualizationAssets(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
