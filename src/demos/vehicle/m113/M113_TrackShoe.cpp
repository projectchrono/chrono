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

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "m113/M113_TrackShoe.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_TrackShoe::m_shoe_height = 0.06;
const double M113_TrackShoe::m_shoe_pitch = 0.154;
const double M113_TrackShoe::m_shoe_mass = 18.02;
const chrono::ChVector<> M113_TrackShoe::m_shoe_inertia(0.22, 0.04, 0.25);

const double M113_TrackShoe::m_cyl_radius = 0.015;
const double M113_TrackShoe::m_front_cyl_loc = 0.0535;
const double M113_TrackShoe::m_rear_cyl_loc = -0.061;

const std::string M113_TrackShoe::m_meshName = "TrackShoe_POV_geom";
const std::string M113_TrackShoe::m_meshFile = vehicle::GetDataFile("M113/TrackShoe.obj");

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_TrackShoe::M113_TrackShoe() : ChSinglePinShoe("M113_TrackShoe"), m_vis_type(PRIMITIVES) {
    SetContactMaterial(0.8f, 0.1f, 1e7f, 0.3f);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_TrackShoe::AddShoeContact() {
    m_shoe->GetCollisionModel()->ClearModel();
    m_shoe->GetCollisionModel()->AddBox(0.055, 0.095, 0.03);
    m_shoe->GetCollisionModel()->AddBox(0.0142, 0.0055, 0.0375, ChVector<>(0.045, 0, 0.0375));
    m_shoe->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_TrackShoe::AddShoeVisualization() {
    switch (m_vis_type) {
        case PRIMITIVES: {
            auto rev_axis = std::make_shared<ChCylinderShape>();
            rev_axis->GetCylinderGeometry().p1 = ChVector<>(0.077, -0.15, 0);
            rev_axis->GetCylinderGeometry().p2 = ChVector<>(0.077, 0.15, 0);
            rev_axis->GetCylinderGeometry().rad = 0.01;
            m_shoe->AddAsset(rev_axis);

            auto cyl_FR = std::make_shared<ChCylinderShape>();
            cyl_FR->GetCylinderGeometry().p1 = ChVector<>(m_front_cyl_loc, -0.1402, 0);
            cyl_FR->GetCylinderGeometry().p2 = ChVector<>(m_front_cyl_loc, -0.0512, 0);
            cyl_FR->GetCylinderGeometry().rad = m_cyl_radius;
            m_shoe->AddAsset(cyl_FR);

            auto cyl_RR = std::make_shared<ChCylinderShape>();
            cyl_RR->GetCylinderGeometry().p1 = ChVector<>(m_rear_cyl_loc, -0.1402, 0);
            cyl_RR->GetCylinderGeometry().p2 = ChVector<>(m_rear_cyl_loc, -0.0512, 0);
            cyl_RR->GetCylinderGeometry().rad = m_cyl_radius;
            m_shoe->AddAsset(cyl_RR);

            auto cyl_FL = std::make_shared<ChCylinderShape>();
            cyl_FL->GetCylinderGeometry().p1 = ChVector<>(m_front_cyl_loc, 0.1402, 0);
            cyl_FL->GetCylinderGeometry().p2 = ChVector<>(m_front_cyl_loc, 0.0512, 0);
            cyl_FL->GetCylinderGeometry().rad = m_cyl_radius;
            m_shoe->AddAsset(cyl_FL);

            auto cyl_RL = std::make_shared<ChCylinderShape>();
            cyl_RL->GetCylinderGeometry().p1 = ChVector<>(m_rear_cyl_loc, 0.1402, 0);
            cyl_RL->GetCylinderGeometry().p2 = ChVector<>(m_rear_cyl_loc, 0.0512, 0);
            cyl_RL->GetCylinderGeometry().rad = m_cyl_radius;
            m_shoe->AddAsset(cyl_RL);

            auto box_shoe = std::make_shared<ChBoxShape>();
            box_shoe->GetBoxGeometry().SetLengths(ChVector<>(0.11, 0.19, 0.06));
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
            trimesh.LoadWavefrontMesh(m_meshFile, false, false);

            auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(m_meshName);
            m_shoe->AddAsset(trimesh_shape);

            break;
        }
    }
}

}  // end namespace m113
