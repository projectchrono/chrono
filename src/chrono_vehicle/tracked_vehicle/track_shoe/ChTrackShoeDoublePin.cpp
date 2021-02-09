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
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeDoublePin.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackShoeDoublePin::ChTrackShoeDoublePin(const std::string& name) : ChTrackShoeSegmented(name) {}

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
    m_geometry.AddCollisionShapes(m_shoe, TrackedCollisionFamily::SHOES);
    m_shoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::SHOES);
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
void ChTrackShoeDoublePin::AddVisualizationAssets(VisualizationType vis) {
    ChTrackShoeSegmented::AddVisualizationAssets(vis);
    AddConnectorVisualization(m_connector_L, vis);
    AddConnectorVisualization(m_connector_R, vis);
}

void ChTrackShoeDoublePin::RemoveVisualizationAssets() {
    ChTrackShoeSegmented::RemoveVisualizationAssets();
    m_connector_L->GetAssets().clear();
    m_connector_R->GetAssets().clear();
}

void ChTrackShoeDoublePin::AddConnectorVisualization(std::shared_ptr<ChBody> connector, VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

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
void ChTrackShoeDoublePin::Connect(std::shared_ptr<ChTrackShoe> next, ChTrackAssembly* assembly, bool ccw) {
    auto track = static_cast<ChTrackAssemblyDoublePin*>(assembly);
    ChSystem* system = m_shoe->GetSystem();
    double sign = ccw ? +1 : -1;

    bool add_RSDA = (track->GetConnectionType() == ChTrackAssemblySegmented::ConnectionType::RSDA_JOINT);
    assert(!add_RSDA || track->GetTorqueFunctor());

    // Create and initialize the revolute joints between shoe body and connector bodies.
    ChVector<> loc_L =
        m_shoe->TransformPointLocalToParent(ChVector<>(sign * GetShoeLength() / 2, +GetShoeWidth() / 2, 0));
    ChVector<> loc_R =
        m_shoe->TransformPointLocalToParent(ChVector<>(sign * GetShoeLength() / 2, -GetShoeWidth() / 2, 0));
    ChQuaternion<> rot = m_shoe->GetRot() * Q_from_AngX(CH_C_PI_2);

    m_revolute_L = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute_L->SetNameString(m_name + "_rev_L");
    m_revolute_L->Initialize(m_shoe, m_connector_L, ChCoordsys<>(loc_L, rot));
    system->AddLink(m_revolute_L);

    m_revolute_R = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute_R->SetNameString(m_name + "_rev_R");
    m_revolute_R->Initialize(m_shoe, m_connector_R, ChCoordsys<>(loc_R, rot));
    system->AddLink(m_revolute_R);

    // Optionally, include rotational spring-dampers to model track bending stiffness
    if (add_RSDA) {
        auto rsda_L = chrono_types::make_shared<ChLinkRotSpringCB>();
        rsda_L->SetNameString(m_name + "_rsda_rev_L");
        rsda_L->Initialize(m_shoe, m_connector_L, false, ChCoordsys<>(loc_L, m_shoe->GetRot()),
                         ChCoordsys<>(loc_L, m_connector_L->GetRot()));
        rsda_L->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(rsda_L);

        auto rsda_R = chrono_types::make_shared<ChLinkRotSpringCB>();
        rsda_R->SetNameString(m_name + "_rsda_rev_R");
        rsda_R->Initialize(m_shoe, m_connector_R, false, ChCoordsys<>(loc_R, m_shoe->GetRot()),
                           ChCoordsys<>(loc_R, m_connector_R->GetRot()));
        rsda_R->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(rsda_R);
    }

    loc_L = m_connector_L->TransformPointLocalToParent(ChVector<>(sign * GetConnectorLength() / 2, 0, 0));
    loc_R = m_connector_R->TransformPointLocalToParent(ChVector<>(sign * GetConnectorLength() / 2, 0, 0));

    // Create connections between these connector bodies and the next shoe body
    if (m_index == -1) {
        // Create and initialize a point-line joint for left connector (sliding along X axis)
        rot = m_connector_L->GetRot() * Q_from_AngZ(CH_C_PI_2);

        auto pointline_L = chrono_types::make_shared<ChLinkLockPointLine>();
        pointline_L->SetNameString(m_name + "_pointline_L");
        pointline_L->Initialize(next->GetShoeBody(), m_connector_L, ChCoordsys<>(loc_L, rot));
        system->AddLink(pointline_L);

        // Create and initialize a point-line joint for left connector (sliding along X axis)
        rot = m_connector_R->GetRot() * Q_from_AngZ(CH_C_PI_2);

        auto pointline_R = chrono_types::make_shared<ChLinkLockPointLine>();
        pointline_R->SetNameString(m_name + "_pointline_R");
        pointline_R->Initialize(next->GetShoeBody(), m_connector_R, ChCoordsys<>(loc_R, rot));
        system->AddLink(pointline_R);
    } else {
        // Create and initialize a spherical joint for left connector
        auto sph_L = chrono_types::make_shared<ChLinkLockSpherical>();
        sph_L->SetNameString(m_name + "_sph_L");
        sph_L->Initialize(next->GetShoeBody(), m_connector_L, ChCoordsys<>(loc_L));
        system->AddLink(sph_L);

        // Create and initialize a spherical joint for left connector
        auto sph_R = chrono_types::make_shared<ChLinkLockSpherical>();
        sph_R->SetNameString(m_name + "_sph_R");
        sph_R->Initialize(next->GetShoeBody(), m_connector_R, ChCoordsys<>(loc_R));
        system->AddLink(sph_R);
    }

    // Optionally, include rotational spring-dampers to model track bending stiffness
    if (add_RSDA) {
        auto rsda_L = chrono_types::make_shared<ChLinkRotSpringCB>();
        rsda_L->SetNameString(m_name + "_rsda_sph_L");
        rsda_L->Initialize(next->GetShoeBody(), m_connector_L, false,
                           ChCoordsys<>(loc_L, next->GetShoeBody()->GetRot()),
                           ChCoordsys<>(loc_L, m_connector_L->GetRot()));
        rsda_L->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(rsda_L);

        auto rsda_R = chrono_types::make_shared<ChLinkRotSpringCB>();
        rsda_R->SetNameString(m_name + "_rsda_sph_R");
        rsda_R->Initialize(next->GetShoeBody(), m_connector_R, false,
                           ChCoordsys<>(loc_R, next->GetShoeBody()->GetRot()),
                           ChCoordsys<>(loc_R, m_connector_R->GetRot()));
        rsda_R->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(rsda_R);
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
