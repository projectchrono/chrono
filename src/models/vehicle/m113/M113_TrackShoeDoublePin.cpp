// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#include "models/vehicle/m113/M113_TrackShoeDoublePin.h"

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
const double M113_TrackShoeDoublePin::m_connector_radius = 0.0223;             // 0.88''
const double M113_TrackShoeDoublePin::m_connector_length = 0.054;              // 2.125''

const std::string M113_TrackShoeDoublePin::m_meshName = "TrackShoe_POV_geom";
const std::string M113_TrackShoeDoublePin::m_meshFile = "M113/TrackShoe.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_TrackShoeDoublePin::M113_TrackShoeDoublePin() : ChTrackShoeDoublePin("M113_TrackShoe"), m_vis_type(PRIMITIVES) {
    SetContactMaterial(0.8f, 0.1f, 1e7f, 0.3f);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_TrackShoeDoublePin::AddShoeContact() {
    m_shoe->GetCollisionModel()->ClearModel();
    m_shoe->GetCollisionModel()->AddBox(0.055, 0.095, 0.03);
    m_shoe->GetCollisionModel()->AddBox(0.0142, 0.0055, 0.0375, ChVector<>(0.045, 0, 0.0375));
    m_shoe->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_TrackShoeDoublePin::AddShoeVisualization() {
    switch (m_vis_type) {
        case PRIMITIVES: {
            auto rev_rear = std::make_shared<ChCylinderShape>();
            rev_rear->GetCylinderGeometry().p1 = ChVector<>(-0.5 * GetShoeLength(), -0.163, 0);
            rev_rear->GetCylinderGeometry().p2 = ChVector<>(-0.5 * GetShoeLength(), +0.163, 0);
            rev_rear->GetCylinderGeometry().rad = 0.01;
            m_shoe->AddAsset(rev_rear);

            auto rev_front = std::make_shared<ChCylinderShape>();
            rev_front->GetCylinderGeometry().p1 = ChVector<>(0.5 * GetShoeLength(), -0.163, 0);
            rev_front->GetCylinderGeometry().p2 = ChVector<>(0.5 * GetShoeLength(), +0.163, 0);
            rev_front->GetCylinderGeometry().rad = 0.01;
            m_shoe->AddAsset(rev_front);

            auto box_shoe = std::make_shared<ChBoxShape>();
            box_shoe->GetBoxGeometry().SetLengths(ChVector<>(GetShoeLength(), 0.23, GetHeight()));
            box_shoe->GetBoxGeometry().Pos = ChVector<>(0, 0, 0);
            m_shoe->AddAsset(box_shoe);

            auto box_pin = std::make_shared<ChBoxShape>();
            box_pin->GetBoxGeometry().SetLengths(ChVector<>(0.0284, 0.0114, 0.075));
            box_pin->GetBoxGeometry().Pos = ChVector<>(0.045, 0, 0.0375);
            m_shoe->AddAsset(box_pin);

            auto col = std::make_shared<ChColorAsset>();
            if (m_index == 0)
                col->SetColor(ChColor(0.6f, 0.3f, 0.3f));
            else if (m_index % 2 == 0)
                col->SetColor(ChColor(0.3f, 0.6f, 0.3f));
            else
                col->SetColor(ChColor(0.3f, 0.3f, 0.6f));
            m_shoe->AddAsset(col);

            break;
        }
        case MESH: {
            geometry::ChTriangleMeshConnected trimesh;
            trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);

            auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(m_meshName);
            m_shoe->AddAsset(trimesh_shape);

            break;
        }
    }
}

void M113_TrackShoeDoublePin::AddConnectorVisualization(std::shared_ptr<ChBody> connector) {
    switch (m_vis_type) {
        case PRIMITIVES: {
            auto cyl_rear = std::make_shared<ChCylinderShape>();
            cyl_rear->GetCylinderGeometry().p1 = ChVector<>(-0.5 * GetConnectorLength(), -0.01, 0);
            cyl_rear->GetCylinderGeometry().p2 = ChVector<>(-0.5 * GetConnectorLength(), +0.01, 0);
            cyl_rear->GetCylinderGeometry().rad = GetConnectorRadius();
            connector->AddAsset(cyl_rear);

            auto cyl_front = std::make_shared<ChCylinderShape>();
            cyl_front->GetCylinderGeometry().p1 = ChVector<>(0.5 * GetConnectorLength(), -0.01, 0);
            cyl_front->GetCylinderGeometry().p2 = ChVector<>(0.5 * GetConnectorLength(), +0.01, 0);
            cyl_front->GetCylinderGeometry().rad = GetConnectorRadius();
            connector->AddAsset(cyl_front);

            auto box = std::make_shared<ChBoxShape>();
            box->GetBoxGeometry().SetLengths(ChVector<>(GetConnectorLength(), 0.02, 2 * GetConnectorRadius()));
            box->GetBoxGeometry().Pos = ChVector<>(0, 0, 0);
            connector->AddAsset(box);

            auto col = std::make_shared<ChColorAsset>();
            if (m_index == 0)
                col->SetColor(ChColor(0.7f, 0.4f, 0.4f));
            else if (m_index % 2 == 0)
                col->SetColor(ChColor(0.4f, 0.7f, 0.4f));
            else
                col->SetColor(ChColor(0.4f, 0.4f, 0.7f));
            connector->AddAsset(col);

            break;
        }
        case MESH: {
            //// TODO

            break;
        }
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
