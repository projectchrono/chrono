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

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/track_shoe/M113_TrackShoeBandANCF.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

//// TODO: check these values
const double M113_TrackShoeBandANCF::m_tread_mass = 1.8;
const double M113_TrackShoeBandANCF::m_web_mass = 2.0;
const ChVector<> M113_TrackShoeBandANCF::m_tread_inertias(0.015, 0.001, 0.016);
const ChVector<> M113_TrackShoeBandANCF::m_web_inertias(0.01, 0.01, 0.01);

const double M113_TrackShoeBandANCF::m_shoe_height = 0.06;

const double M113_TrackShoeBandANCF::m_belt_width = 0.3175;  // 12.5 in

const double M113_TrackShoeBandANCF::m_tooth_tip_length = 0.0126 * 1.04;
const double M113_TrackShoeBandANCF::m_tooth_base_length = 0.0529 * 1.04;
const double M113_TrackShoeBandANCF::m_tooth_width = 0.0508;  // 2 in
const double M113_TrackShoeBandANCF::m_tooth_height = 0.0385 * 1.04;
const double M113_TrackShoeBandANCF::m_tooth_arc_radius = 0.0540 * 1.04;

const double M113_TrackShoeBandANCF::m_web_length = 0.0335 * 1.04;
const double M113_TrackShoeBandANCF::m_web_thickness = 0.0188 * 1.04;
const double M113_TrackShoeBandANCF::m_steel_thickness = 0.05 * 25.4 / 1000.0;

const double M113_TrackShoeBandANCF::m_tread_length = 0.0724 * 1.04;
const double M113_TrackShoeBandANCF::m_tread_thickness = 0.0157 * 1.04;

const ChVector<> M113_TrackShoeBandANCF::m_guide_box_dims(0.0529, 0.0114, 0.075);
const double M113_TrackShoeBandANCF::m_guide_box_offset_x = 0;

const std::string M113_TrackShoeBandANCF::m_meshFile = "M113/meshes/TrackShoeBand.obj";
const std::string M113_TrackShoeBandANCF::m_tread_meshName = "M113_Tread";

// -----------------------------------------------------------------------------

M113_TrackShoeBandANCF::M113_TrackShoeBandANCF(const std::string& name,
                                               ElementType element_type,
                                               bool constrain_curvature,
                                               int num_elements_length,
                                               int num_elements_width)
    : ChTrackShoeBandANCF(name, element_type, constrain_curvature),
      m_num_elements_length(num_elements_length),
      m_num_elements_width(num_elements_width) {
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

void M113_TrackShoeBandANCF::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh =
            geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(GetDataFile(m_meshFile), false, false);
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(m_meshFile).stem());
        trimesh_shape->SetMutable(false);
        m_shoe->AddVisualShape(trimesh_shape);
    } else {
        ChTrackShoeBandANCF::AddVisualizationAssets(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
