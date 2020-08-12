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
// Base class for a double-pin track shoe (template definition).
//
// =============================================================================

#include "chrono/core/ChGlobal.h"
#include "chrono/assets/ChAssetLevel.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeDoublePin.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackShoeDoublePin::ChTrackShoeDoublePin(const std::string& name) : ChTrackShoe(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeDoublePin::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                      const ChVector<>& location,
                                      const ChQuaternion<>& rotation) {
    ChSystem* sys = chassis->GetSystem();

    // Express the track shoe location and orientation in global frame.
    ChVector<> loc = chassis->TransformPointLocalToParent(location);
    ChQuaternion<> rot = chassis->GetRot() * rotation;
    ChVector<> xdir = rot.GetXaxis();
    ChVector<> ydir = rot.GetYaxis();

    // Create the shoe body
    m_shoe = std::shared_ptr<ChBody>(sys->NewBody());
    m_shoe->SetNameString(m_name + "_shoe");
    m_shoe->SetIdentifier(BodyID::SHOE_BODY);
    m_shoe->SetPos(loc - (0.5 * GetConnectorLength()) * xdir);
    m_shoe->SetRot(rot);
    m_shoe->SetMass(GetShoeMass());
    m_shoe->SetInertiaXX(GetShoeInertia());
    m_shoe->SetCollide(true);
    chassis->GetSystem()->AddBody(m_shoe);

    // Create the connector bodies.
    m_connector_L = std::shared_ptr<ChBody>(sys->NewBody());
    m_connector_L->SetNameString(m_name + "_connector_L");
    m_connector_L->SetPos(loc + (0.5 * GetShoeLength()) * xdir + (0.5 * GetShoeWidth()) * ydir);
    m_connector_L->SetRot(rot);
    m_connector_L->SetMass(GetConnectorMass());
    m_connector_L->SetInertiaXX(GetConnectorInertia());
    chassis->GetSystem()->AddBody(m_connector_L);

    m_connector_R = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_connector_R->SetNameString(m_name + "_connector_R");
    m_connector_R->SetPos(loc + (0.5 * GetShoeLength()) * xdir - (0.5 * GetShoeWidth()) * ydir);
    m_connector_R->SetRot(rot);
    m_connector_R->SetMass(GetConnectorMass());
    m_connector_R->SetInertiaXX(GetConnectorInertia());
    chassis->GetSystem()->AddBody(m_connector_R);

    // Create all contact materials
    CreateContactMaterials(sys->GetContactMethod());

    // Add contact geometry on shoe body
    AddShoeContact();
}

void ChTrackShoeDoublePin::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                      const ChVector<>& loc_shoe,
                                      const ChQuaternion<>& rot_shoe,
                                      const ChVector<>& loc_connector_L,
                                      const ChVector<>& loc_connector_R,
                                      const ChQuaternion<>& rot_connector) {
    // Initialize at origin.
    Initialize(chassis, VNULL, QUNIT);

    // Overwrite absolute body locations and orientations.
    m_shoe->SetPos(chassis->TransformPointLocalToParent(loc_shoe));
    m_shoe->SetRot(chassis->GetRot() * rot_shoe);

    m_connector_L->SetPos(chassis->TransformPointLocalToParent(loc_connector_L));
    m_connector_L->SetRot(chassis->GetRot() * rot_connector);

    m_connector_R->SetPos(chassis->TransformPointLocalToParent(loc_connector_R));
    m_connector_R->SetRot(chassis->GetRot() * rot_connector);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChTrackShoeDoublePin::GetMass() const {
    return GetShoeMass() + 2 * GetConnectorMass();
}

double ChTrackShoeDoublePin::GetPitch() const {
    return GetShoeLength() + GetConnectorLength();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeDoublePin::AddShoeContact() {
    m_shoe->GetCollisionModel()->ClearModel();

    m_shoe->GetCollisionModel()->SetFamily(TrackedCollisionFamily::SHOES);
    m_shoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::SHOES);

    for (auto box : m_coll_boxes) {
        assert(m_shoe_materials[box.m_matID] &&
               m_shoe_materials[box.m_matID]->GetContactMethod() == m_shoe->GetSystem()->GetContactMethod());
        ChVector<> hdims = box.m_dims / 2;
        m_shoe->GetCollisionModel()->AddBox(m_shoe_materials[box.m_matID], hdims.x(), hdims.y(), hdims.z(), box.m_pos,
                                            box.m_rot);
    }
    for (auto cyl : m_coll_cylinders) {
        assert(m_shoe_materials[cyl.m_matID] &&
               m_shoe_materials[cyl.m_matID]->GetContactMethod() == m_shoe->GetSystem()->GetContactMethod());
        m_shoe->GetCollisionModel()->AddCylinder(m_shoe_materials[cyl.m_matID], cyl.m_radius, cyl.m_radius,
                                                 cyl.m_length / 2, cyl.m_pos, cyl.m_rot);
    }

    m_shoe->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeDoublePin::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    AddShoeVisualization();
    AddConnectorVisualization(m_connector_L);
    AddConnectorVisualization(m_connector_R);
}

void ChTrackShoeDoublePin::RemoveVisualizationAssets() {
    m_shoe->GetAssets().clear();
    m_connector_L->GetAssets().clear();
    m_connector_R->GetAssets().clear();
}

void ChTrackShoeDoublePin::AddShoeVisualization() {
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
    for (auto box : m_vis_boxes) {
        auto box_shape = chrono_types::make_shared<ChBoxShape>();
        box_shape->GetBoxGeometry().SetLengths(box.m_dims);
        box_shape->Pos = box.m_pos;
        box_shape->Rot = box.m_rot;
        box_level->AddAsset(box_shape);
    }
    box_level->AddAsset(chrono_types::make_shared<ChColorAsset>(box_col));

    // Render cylinders
    auto cyl_level = chrono_types::make_shared<ChAssetLevel>();
    for (auto cyl : m_vis_cylinders) {
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

void ChTrackShoeDoublePin::AddConnectorVisualization(std::shared_ptr<ChBody> connector) {
    double c_length = GetConnectorLength();
    double c_width = GetConnectorWidth();
    double c_radius = GetConnectorRadius();

    auto cyl_rear = chrono_types::make_shared<ChCylinderShape>();
    cyl_rear->GetCylinderGeometry().p1 = ChVector<>(-0.5 * c_length, -0.5 * c_width, 0);
    cyl_rear->GetCylinderGeometry().p2 = ChVector<>(-0.5 * c_length, +0.5 * c_width, 0);
    cyl_rear->GetCylinderGeometry().rad = c_radius;
    connector->AddAsset(cyl_rear);

    auto cyl_front = chrono_types::make_shared<ChCylinderShape>();
    cyl_front->GetCylinderGeometry().p1 = ChVector<>(0.5 * c_length, -0.5 * c_width, 0);
    cyl_front->GetCylinderGeometry().p2 = ChVector<>(0.5 * c_length, +0.5 * c_width, 0);
    cyl_front->GetCylinderGeometry().rad = c_radius;
    connector->AddAsset(cyl_front);

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().SetLengths(ChVector<>(c_length, c_width, 2 * c_radius));
    box->Pos = ChVector<>(0, 0, 0);
    connector->AddAsset(box);

    auto col = chrono_types::make_shared<ChColorAsset>();
    if (m_index == 0)
        col->SetColor(ChColor(0.7f, 0.4f, 0.4f));
    else if (m_index % 2 == 0)
        col->SetColor(ChColor(0.4f, 0.7f, 0.4f));
    else
        col->SetColor(ChColor(0.4f, 0.4f, 0.7f));
    connector->AddAsset(col);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeDoublePin::Connect(std::shared_ptr<ChTrackShoe> next, bool ccw) {
    ChSystem* system = m_shoe->GetSystem();
    double sign = ccw ? +1 : -1;

    // Create and initialize the revolute joints between shoe body and connector bodies.
    ChVector<> loc_L =
        m_shoe->TransformPointLocalToParent(ChVector<>(sign * GetShoeLength() / 2, +GetShoeWidth() / 2, 0));
    ChVector<> loc_R =
        m_shoe->TransformPointLocalToParent(ChVector<>(sign * GetShoeLength() / 2, -GetShoeWidth() / 2, 0));
    ChQuaternion<> rot = m_shoe->GetRot() * Q_from_AngX(CH_C_PI_2);

    m_revolute_L = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute_L->SetNameString(m_name + "_rev_this_L");
    m_revolute_L->Initialize(m_shoe, m_connector_L, ChCoordsys<>(loc_L, rot));
    system->AddLink(m_revolute_L);

    m_revolute_R = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute_R->SetNameString(m_name + "_rev_this_R");
    m_revolute_R->Initialize(m_shoe, m_connector_R, ChCoordsys<>(loc_R, rot));
    system->AddLink(m_revolute_R);

    // Create connections between these connector bodies and the next shoe body
    if (m_index == -1) {
        // Create and initialize a point-line joint for left connector (sliding along X axis)
        loc_L = m_connector_L->TransformPointLocalToParent(ChVector<>(sign * GetConnectorLength() / 2, 0, 0));
        rot = m_connector_L->GetRot() * Q_from_AngZ(CH_C_PI_2);

        auto pointline_L = chrono_types::make_shared<ChLinkLockPointLine>();
        pointline_L->SetNameString(m_name + "_pointline_next_L");
        pointline_L->Initialize(next->GetShoeBody(), m_connector_L, ChCoordsys<>(loc_L, rot));
        system->AddLink(pointline_L);

        // Create and initialize a point-line joint for left connector (sliding along X axis)
        loc_R = m_connector_R->TransformPointLocalToParent(ChVector<>(sign * GetConnectorLength() / 2, 0, 0));
        rot = m_connector_R->GetRot() * Q_from_AngZ(CH_C_PI_2);

        auto pointline_R = chrono_types::make_shared<ChLinkLockPointLine>();
        pointline_R->SetNameString(m_name + "_pointline_next_R");
        pointline_R->Initialize(next->GetShoeBody(), m_connector_R, ChCoordsys<>(loc_R, rot));
        system->AddLink(pointline_R);
    } else {
        // Create and initialize a spherical joint for left connector
        loc_L = m_connector_L->TransformPointLocalToParent(ChVector<>(sign * GetConnectorLength() / 2, 0, 0));

        auto sph_L = chrono_types::make_shared<ChLinkLockSpherical>();
        sph_L->SetNameString(m_name + "_sph_next_L");
        sph_L->Initialize(next->GetShoeBody(), m_connector_L, ChCoordsys<>(loc_L));
        system->AddLink(sph_L);

        // Create and initialize a spherical joint for left connector
        loc_R = m_connector_R->TransformPointLocalToParent(ChVector<>(sign * GetConnectorLength() / 2, 0, 0));

        auto sph_R = chrono_types::make_shared<ChLinkLockSpherical>();
        sph_R->SetNameString(m_name + "_sph_next_R");
        sph_R->Initialize(next->GetShoeBody(), m_connector_R, ChCoordsys<>(loc_R));
        system->AddLink(sph_R);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeDoublePin::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_shoe);
    bodies.push_back(m_connector_L);
    bodies.push_back(m_connector_R);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute_L);
    joints.push_back(m_revolute_R);
    ChPart::ExportJointList(jsonDocument, joints);
}

void ChTrackShoeDoublePin::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_shoe);
    bodies.push_back(m_connector_L);
    bodies.push_back(m_connector_R);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute_L);
    joints.push_back(m_revolute_R);
    database.WriteJoints(joints);
}

}  // end namespace vehicle
}  // end namespace chrono
