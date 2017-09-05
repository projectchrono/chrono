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

#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/M113_TrackShoeRigidCB.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

//// TODO: check these values
const double M113_TrackShoeRigidCB::m_tread_mass = 18.02;
const double M113_TrackShoeRigidCB::m_web_mass = 2.0;
const ChVector<> M113_TrackShoeRigidCB::m_tread_inertias(0.22, 0.04, 0.25);
const ChVector<> M113_TrackShoeRigidCB::m_web_inertias(0.01, 0.01, 0.01);

const double M113_TrackShoeRigidCB::m_shoe_height = 0.06;

const double M113_TrackShoeRigidCB::m_belt_width = 0.3175;  // 12.5 in

const double M113_TrackShoeRigidCB::m_tooth_tip_length = 0.0126;
const double M113_TrackShoeRigidCB::m_tooth_base_length = 0.0529;
const double M113_TrackShoeRigidCB::m_tooth_width = 0.0508;  // 2 in
const double M113_TrackShoeRigidCB::m_tooth_height = 0.0385;
const double M113_TrackShoeRigidCB::m_tooth_arc_radius = 0.0540;
const ChVector2<> M113_TrackShoeRigidCB::m_tooth_arc_center(-0.027424882274996, -0.003673834489381);

const int M113_TrackShoeRigidCB::m_num_web_segments = 1;
const double M113_TrackShoeRigidCB::m_web_length = 0.0335;
const double M113_TrackShoeRigidCB::m_web_thickness = 0.0188;

const double M113_TrackShoeRigidCB::m_tread_length = 0.0724;
const double M113_TrackShoeRigidCB::m_tread_thickness = 0.0157;

const ChVector<> M113_TrackShoeRigidCB::m_guide_box_dims(0.0529, 0.0114, 0.1016);
const double M113_TrackShoeRigidCB::m_guide_box_offset_x = 0;

const std::string M113_TrackShoeRigidCB::m_meshName = "TrackShoeRigidCB_POV_geom";
const std::string M113_TrackShoeRigidCB::m_meshFile = "M113/TrackShoeRigidCB.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_TrackShoeRigidCB::M113_TrackShoeRigidCB() : ChTrackShoeRigidCB("M113_TrackShoe") {
    SetContactFrictionCoefficient(0.8f);
    SetContactRestitutionCoefficient(0.1f);
    SetContactMaterialProperties(1e7f, 0.3f);
    SetContactMaterialCoefficients(2e5f, 40.0f, 2e5f, 20.0f);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_TrackShoeRigidCB::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        //// TODO:
        //// Set up meshes for tread body and web links.
        //// For now, default to PRIMITIVE visualization
        ChTrackShoeRigidCB::AddVisualizationAssets(vis);
    } else {
        ChTrackShoeRigidCB::AddVisualizationAssets(vis);
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
