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
// Base class for segmented track shoes.
//
// =============================================================================

#include "chrono/core/ChGlobal.h"
#include "chrono/assets/ChAssetLevel.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSegmented.h"

namespace chrono {
namespace vehicle {

ChTrackShoeSegmented::ChTrackShoeSegmented(const std::string& name) : ChTrackShoe(name) {}

void ChTrackShoeSegmented::AddShoeContact() {
    m_shoe->GetCollisionModel()->ClearModel();

    m_shoe->GetCollisionModel()->SetFamily(TrackedCollisionFamily::SHOES);
    m_shoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::SHOES);

    for (const auto& box : m_coll_boxes) {
        assert(m_shoe_materials[box.m_matID] &&
               m_shoe_materials[box.m_matID]->GetContactMethod() == m_shoe->GetSystem()->GetContactMethod());
        ChVector<> hdims = box.m_dims / 2;
        m_shoe->GetCollisionModel()->AddBox(m_shoe_materials[box.m_matID], hdims.x(), hdims.y(), hdims.z(), box.m_pos,
                                            box.m_rot);
    }
    for (const auto& cyl : m_coll_cylinders) {
        assert(m_shoe_materials[cyl.m_matID] &&
               m_shoe_materials[cyl.m_matID]->GetContactMethod() == m_shoe->GetSystem()->GetContactMethod());
        m_shoe->GetCollisionModel()->AddCylinder(m_shoe_materials[cyl.m_matID], cyl.m_radius, cyl.m_radius,
                                                 cyl.m_length / 2, cyl.m_pos, cyl.m_rot);
    }

    m_shoe->GetCollisionModel()->BuildModel();
}

void ChTrackShoeSegmented::AddShoeVisualization() {
    // Create colors for the track shoe (based on shoe index)
    ChColor box_col;
    ChColor cyl_col;
    if (m_index == 0) {
        box_col = {0.6f, 0.3f, 0.3f};
        cyl_col = {0.4f, 0.1f, 0.1f};
    } else if (m_index % 2 == 0) {
        box_col = {0.3f, 0.6f, 0.3f};
        cyl_col = {0.1f, 0.4f, 0.1f};
    } else {
        box_col = {0.3f, 0.3f, 0.6f};
        cyl_col = {0.1f, 0.1f, 0.4f};
    }

    // Render boxes
    auto box_level = chrono_types::make_shared<ChAssetLevel>();
    for (const auto& box : m_vis_boxes) {
        auto box_shape = chrono_types::make_shared<ChBoxShape>();
        box_shape->GetBoxGeometry().SetLengths(box.m_dims);
        box_shape->Pos = box.m_pos;
        box_shape->Rot = box.m_rot;
        box_level->AddAsset(box_shape);
    }
    box_level->AddAsset(chrono_types::make_shared<ChColorAsset>(box_col));

    // Render cylinders
    auto cyl_level = chrono_types::make_shared<ChAssetLevel>();
    for (const auto& cyl : m_vis_cylinders) {
        auto cyl_shape = chrono_types::make_shared<ChCylinderShape>();
        cyl_shape->GetCylinderGeometry().rad = cyl.m_radius;
        cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, cyl.m_length / 2, 0);
        cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, -cyl.m_length / 2, 0);
        cyl_shape->Pos = cyl.m_pos;
        cyl_shape->Rot = cyl.m_rot;
        cyl_level->AddAsset(cyl_shape);
    }
    cyl_level->AddAsset(chrono_types::make_shared<ChColorAsset>(cyl_col));

    // Attach asset levels
    m_shoe->AddAsset(box_level);
    m_shoe->AddAsset(cyl_level);
}


}  // end namespace vehicle
}  // end namespace chrono
