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
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeDoublePin.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChTrackShoeDoublePin::ChTrackShoeDoublePin(const std::string& name) : ChTrackShoeSegmented(name) {}

ChTrackShoeDoublePin::~ChTrackShoeDoublePin() {
    if (!m_shoe)
        return;

    auto sys = m_shoe->GetSystem();
    if (sys) {
        sys->Remove(m_connector_L);
        sys->Remove(m_connector_R);

        ChChassis::RemoveJoint(m_revolute_L);
        ChChassis::RemoveJoint(m_revolute_R);
        if (m_rsda_L) {
            sys->Remove(m_rsda_L);
            sys->Remove(m_rsda_R);
        }

        ChChassis::RemoveJoint(m_connection_joint_L);
        ChChassis::RemoveJoint(m_connection_joint_R);
        if (m_connection_rsda_L) {
            sys->Remove(m_connection_rsda_L);
            sys->Remove(m_connection_rsda_R);
        }
    }
}

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

void ChTrackShoeDoublePin::InitializeInertiaProperties() {
    m_mass = GetShoeMass() + 2 * GetConnectorMass();
}

void ChTrackShoeDoublePin::UpdateInertiaProperties() {
    m_xform = m_shoe->GetFrame_REF_to_abs();

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;
    composite.AddComponent(m_shoe->GetFrame_COG_to_abs(), m_shoe->GetMass(), m_shoe->GetInertia());
    composite.AddComponent(m_connector_L->GetFrame_COG_to_abs(), m_connector_L->GetMass(), m_connector_L->GetInertia());
    composite.AddComponent(m_connector_R->GetFrame_COG_to_abs(), m_connector_R->GetMass(), m_connector_R->GetInertia());

    // Express COM and inertia in subsystem reference frame
    m_com.coord.pos = m_xform.TransformPointParentToLocal(composite.GetCOM());
    m_com.coord.rot = QUNIT;

    m_inertia = m_xform.GetA().transpose() * composite.GetInertia() * m_xform.GetA();
}

double ChTrackShoeDoublePin::GetPitch() const {
    return GetShoeLength() + GetConnectorLength();
}

// -----------------------------------------------------------------------------
void ChTrackShoeDoublePin::AddVisualizationAssets(VisualizationType vis) {
    ChTrackShoeSegmented::AddVisualizationAssets(vis);
    AddConnectorVisualization(m_connector_L, vis);
    AddConnectorVisualization(m_connector_R, vis);
}

void ChTrackShoeDoublePin::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_connector_L);
    ChPart::RemoveVisualizationAssets(m_connector_R);
    ChTrackShoeSegmented::RemoveVisualizationAssets();
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
    connector->AddVisualShape(cyl_rear);

    auto cyl_front = chrono_types::make_shared<ChCylinderShape>();
    cyl_front->GetCylinderGeometry().p1 = ChVector<>(0.5 * c_length, -0.5 * c_width, 0);
    cyl_front->GetCylinderGeometry().p2 = ChVector<>(0.5 * c_length, +0.5 * c_width, 0);
    cyl_front->GetCylinderGeometry().rad = c_radius;
    connector->AddVisualShape(cyl_front);

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().SetLengths(ChVector<>(c_length, c_width, 2 * c_radius));
    connector->AddVisualShape(box, ChFrame<>());
}

// -----------------------------------------------------------------------------

void ChTrackShoeDoublePin::EnableTrackBendingStiffness(bool val) {
    m_rsda_L->SetDisabled(val);
    m_rsda_R->SetDisabled(val);
    m_connection_rsda_L->SetDisabled(val);
    m_connection_rsda_R->SetDisabled(val);
}

void ChTrackShoeDoublePin::Connect(std::shared_ptr<ChTrackShoe> next,
                                   ChTrackAssembly* assembly,
                                   ChChassis* chassis,
                                   bool ccw) {
    auto track = static_cast<ChTrackAssemblyDoublePin*>(assembly);
    ChSystem* system = m_shoe->GetSystem();
    double sign = ccw ? +1 : -1;

    ChVector<> pShoe_L;     // local point on shoe (left)
    ChVector<> pShoe_R;     // local point on shoe (right)
    ChVector<> pConnector;  // local point on connector

    ChVector<> loc_L;    // left point (expressed in absolute frame)
    ChVector<> loc_R;    // right point (expressed in absolute frame)
    ChQuaternion<> rot;  // orientation (expressed in absolute frame)

    // 1. Connections between this shoe body and connector bodies

    // Create and initialize the revolute joints/bushings between shoe body and connector bodies.
    pShoe_L = ChVector<>(sign * GetShoeLength() / 2, +GetShoeWidth() / 2, 0);
    pShoe_R = ChVector<>(sign * GetShoeLength() / 2, -GetShoeWidth() / 2, 0);
    pConnector = ChVector<>(-sign * GetConnectorLength() / 2, 0, 0);

    loc_L = m_shoe->TransformPointLocalToParent(pShoe_L);
    loc_R = m_shoe->TransformPointLocalToParent(pShoe_R);
    rot = m_shoe->GetRot() * Q_from_AngX(CH_C_PI_2);

    m_revolute_L =
        chrono_types::make_shared<ChVehicleJoint>(ChVehicleJoint::Type::REVOLUTE, m_name + "_pin_L", m_shoe,
                                                  m_connector_L, ChCoordsys<>(loc_L, rot), track->GetBushingData());
    chassis->AddJoint(m_revolute_L);

    m_revolute_R =
        chrono_types::make_shared<ChVehicleJoint>(ChVehicleJoint::Type::REVOLUTE, m_name + "_pin_R", m_shoe,
                                                  m_connector_R, ChCoordsys<>(loc_R, rot), track->GetBushingData());
    chassis->AddJoint(m_revolute_R);

    // Optionally, include rotational spring-dampers to model track bending stiffness.
    // The RSDA frames are aligned with the corresponding body frames and the springs have a default zero rest angle.
    if (track->GetTorqueFunctor()) {
        ChQuaternion<> z2y = Q_from_AngX(-CH_C_PI_2);

        m_rsda_L = chrono_types::make_shared<ChLinkRSDA>();
        m_rsda_L->SetNameString(m_name + "_rsda_pin_L");
        m_rsda_L->Initialize(m_shoe, m_connector_L, true, ChCoordsys<>(pShoe_L, z2y), ChCoordsys<>(pConnector, z2y));
        m_rsda_L->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(m_rsda_L);

        m_rsda_R = chrono_types::make_shared<ChLinkRSDA>();
        m_rsda_R->SetNameString(m_name + "_rsda_pin_R");
        m_rsda_R->Initialize(m_shoe, m_connector_R, true, ChCoordsys<>(pShoe_R, z2y), ChCoordsys<>(pConnector, z2y));
        m_rsda_R->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(m_rsda_R);
    }

    // 2. Connections between connector bodies and next shoe body

    pShoe_L = ChVector<>(-sign * GetShoeLength() / 2, +GetShoeWidth() / 2, 0);
    pShoe_R = ChVector<>(-sign * GetShoeLength() / 2, -GetShoeWidth() / 2, 0);
    pConnector = ChVector<>(sign * GetConnectorLength() / 2, 0, 0);

    loc_L = m_connector_L->TransformPointLocalToParent(pConnector);
    loc_R = m_connector_R->TransformPointLocalToParent(pConnector);

    if (!track->GetBushingData()) {
        // Connect left connector
        if (m_index == 0) {
            // Pointline joint (sliding along X)
            rot = m_connector_L->GetRot() * Q_from_AngZ(CH_C_PI_2);
            m_connection_joint_L =
                chrono_types::make_shared<ChVehicleJoint>(ChVehicleJoint::Type::POINTLINE, m_name + "_cpin_L",
                                                          m_connector_L, next->GetShoeBody(), ChCoordsys<>(loc_L, rot));
            chassis->AddJoint(m_connection_joint_L);
        } else {
            // Revolute joint
            rot = m_connector_L->GetRot() * Q_from_AngX(CH_C_PI_2);
            m_connection_joint_L =
                chrono_types::make_shared<ChVehicleJoint>(ChVehicleJoint::Type::REVOLUTE, m_name + "_cpin_L",
                                                          m_connector_L, next->GetShoeBody(), ChCoordsys<>(loc_L, rot));
            chassis->AddJoint(m_connection_joint_L);
        }

        // Connect right connector through a point-plane joint (normal along Z axis)
        rot = m_connector_R->GetRot();
        m_connection_joint_R =
            chrono_types::make_shared<ChVehicleJoint>(ChVehicleJoint::Type::POINTPLANE, m_name + "_cpin_R",
                                                      m_connector_R, next->GetShoeBody(), ChCoordsys<>(loc_R, rot));
        chassis->AddJoint(m_connection_joint_R);
    } else {
        // Create and initialize revolute bushings between connector bodies and next shoe body.
        rot = m_connector_L->GetRot() * Q_from_AngX(CH_C_PI_2);
        m_connection_joint_L = chrono_types::make_shared<ChVehicleJoint>(
            ChVehicleJoint::Type::REVOLUTE, m_name + "_cpin_L", m_connector_L, next->GetShoeBody(),
            ChCoordsys<>(loc_L, rot), track->GetBushingData());
        chassis->AddJoint(m_connection_joint_L);

        rot = m_connector_R->GetRot() * Q_from_AngX(CH_C_PI_2);
        m_connection_joint_R = chrono_types::make_shared<ChVehicleJoint>(
            ChVehicleJoint::Type::REVOLUTE, m_name + "_cpin_R", m_connector_R, next->GetShoeBody(),
            ChCoordsys<>(loc_R, rot), track->GetBushingData());
        chassis->AddJoint(m_connection_joint_R);
    }

    // Optionally, include rotational spring-dampers to model track bending stiffness
    // The RSDA frames are aligned with the corresponding body frames and the springs has a default zero rest angle.
    if (track->GetTorqueFunctor()) {
        ChQuaternion<> z2y = Q_from_AngX(-CH_C_PI_2);

        m_connection_rsda_L = chrono_types::make_shared<ChLinkRSDA>();
        m_connection_rsda_L->SetNameString(m_name + "_rsda_cpin_L");
        m_connection_rsda_L->Initialize(m_connector_L, next->GetShoeBody(), true, ChCoordsys<>(pConnector, z2y),
                                        ChCoordsys<>(pShoe_L, z2y));
        m_connection_rsda_L->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(m_connection_rsda_L);

        m_connection_rsda_R = chrono_types::make_shared<ChLinkRSDA>();
        m_connection_rsda_R->SetNameString(m_name + "_rsda_cpin_R");
        m_connection_rsda_R->Initialize(m_connector_R, next->GetShoeBody(), true, ChCoordsys<>(pConnector, z2y),
                                        ChCoordsys<>(pShoe_R, z2y));
        m_connection_rsda_R->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(m_connection_rsda_R);
    }
}

// -----------------------------------------------------------------------------
void ChTrackShoeDoublePin::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_shoe);
    bodies.push_back(m_connector_L);
    bodies.push_back(m_connector_R);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    m_revolute_L->IsKinematic() ? joints.push_back(m_revolute_L->GetAsLink())
                                : bushings.push_back(m_revolute_L->GetAsBushing());
    m_revolute_R->IsKinematic() ? joints.push_back(m_revolute_R->GetAsLink())
                                : bushings.push_back(m_revolute_R->GetAsBushing());
    ChPart::ExportJointList(jsonDocument, joints);
    ChPart::ExportBodyLoadList(jsonDocument, bushings);
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
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    m_revolute_L->IsKinematic() ? joints.push_back(m_revolute_L->GetAsLink())
                                : bushings.push_back(m_revolute_L->GetAsBushing());
    m_revolute_R->IsKinematic() ? joints.push_back(m_revolute_R->GetAsLink())
                                : bushings.push_back(m_revolute_R->GetAsBushing());
    database.WriteJoints(joints);
    database.WriteBodyLoads(bushings);
}

}  // end namespace vehicle
}  // end namespace chrono
