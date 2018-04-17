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

const double M113_TrackShoeDoublePin::m_connector_mass = 2.0;                  //// TODO
const ChVector<> M113_TrackShoeDoublePin::m_connector_inertia(0.1, 0.1, 0.1);  //// TODO
const double M113_TrackShoeDoublePin::m_connector_radius = 0.02;               // 0.88''
const double M113_TrackShoeDoublePin::m_connector_length = 0.054;              // 2.125''
const double M113_TrackShoeDoublePin::m_connector_width = 0.02;

const ChVector<> M113_TrackShoeDoublePin::m_pad_box_dims(0.11, 0.19, 0.06);
const ChVector<> M113_TrackShoeDoublePin::m_pad_box_loc(0, 0, 0);
const ChVector<> M113_TrackShoeDoublePin::m_guide_box_dims(0.0284, 0.0114, 0.075);
const ChVector<> M113_TrackShoeDoublePin::m_guide_box_loc(0.045, 0, 0.0375);

const std::string M113_TrackShoeDoublePin::m_meshName = "TrackShoeDoublePin_POV_geom";
const std::string M113_TrackShoeDoublePin::m_meshFile = "M113/TrackShoeDoublePin.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_TrackShoeDoublePin::M113_TrackShoeDoublePin(const std::string& name) : ChTrackShoeDoublePin(name) {
    SetContactFrictionCoefficient(0.8f);
    SetContactRestitutionCoefficient(0.1f);
    SetContactMaterialProperties(1e7f, 0.3f);
    SetContactMaterialCoefficients(2e5f, 40.0f, 2e5f, 20.0f);
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
