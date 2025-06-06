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
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeDoublePin.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChTrackShoeDoublePin::ChTrackShoeDoublePin(const std::string& name, DoublePinTrackShoeType topology)
    : ChTrackShoeSegmented(name), m_topology(topology) {}

ChTrackShoeDoublePin::~ChTrackShoeDoublePin() {
    if (!m_shoe)
        return;

    auto sys = m_shoe->GetSystem();
    if (sys) {
        switch (m_topology) {
            case DoublePinTrackShoeType::TWO_CONNECTORS:
                sys->Remove(m_connector_L);
                sys->Remove(m_connector_R);

                ChChassis::RemoveJoint(m_joint_L);
                ChChassis::RemoveJoint(m_joint_R);
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
                break;
            case DoublePinTrackShoeType::ONE_CONNECTOR:
                sys->Remove(m_connector_L);
                ChChassis::RemoveJoint(m_joint_L);
                if (m_rsda_L) {
                    sys->Remove(m_rsda_L);
                }
                ChChassis::RemoveJoint(m_connection_joint_L);
                if (m_connection_rsda_L) {
                    sys->Remove(m_connection_rsda_L);
                }
                break;
        }
    }
}

// -----------------------------------------------------------------------------
void ChTrackShoeDoublePin::Construct(std::shared_ptr<ChChassis> chassis,
                                     const ChVector3d& location,
                                     const ChQuaternion<>& rotation) {
    ChTrackShoeSegmented::Construct(chassis, location, rotation);

    ChSystem* sys = chassis->GetSystem();

    // Express the track shoe location and orientation in global frame.
    auto chassis_body = chassis->GetBody();
    ChVector3d loc = chassis_body->TransformPointLocalToParent(location);
    ChQuaternion<> rot = chassis_body->GetRot() * rotation;
    ChVector3d xdir = rot.GetAxisX();
    ChVector3d ydir = rot.GetAxisY();

    // Create the shoe body
    m_shoe = chrono_types::make_shared<ChBody>();
    m_shoe->SetName(m_name + "_shoe");
    m_shoe->SetTag(m_obj_tag);
    m_shoe->SetTag(m_obj_tag);
    m_shoe->SetPos(loc - (0.5 * GetConnectorLength()) * xdir);
    m_shoe->SetRot(rot);
    m_shoe->SetMass(GetShoeMass());
    m_shoe->SetInertiaXX(GetShoeInertia());
    m_shoe->EnableCollision(true);
    chassis->GetSystem()->AddBody(m_shoe);

    // Create the connector bodies.
    switch (m_topology) {
        case DoublePinTrackShoeType::TWO_CONNECTORS: {
            m_connector_L = chrono_types::make_shared<ChBody>();
            m_connector_L->SetName(m_name + "_connector_L");
            m_connector_L->SetTag(m_obj_tag);
            m_connector_L->SetPos(loc + (0.5 * GetShoeLength()) * xdir + (0.5 * GetShoeWidth()) * ydir);
            m_connector_L->SetRot(rot);
            m_connector_L->SetMass(GetConnectorMass());
            m_connector_L->SetInertiaXX(GetConnectorInertia());
            chassis->GetSystem()->AddBody(m_connector_L);

            m_connector_R = chrono_types::make_shared<ChBody>();
            m_connector_R->SetName(m_name + "_connector_R");
            m_connector_R->SetTag(m_obj_tag);
            m_connector_R->SetPos(loc + (0.5 * GetShoeLength()) * xdir - (0.5 * GetShoeWidth()) * ydir);
            m_connector_R->SetRot(rot);
            m_connector_R->SetMass(GetConnectorMass());
            m_connector_R->SetInertiaXX(GetConnectorInertia());
            chassis->GetSystem()->AddBody(m_connector_R);
        } break;
        case DoublePinTrackShoeType::ONE_CONNECTOR: {
            // Set mass and inertia corresponding to two separate connector bodies.
            double mass_connector = 2.0 * GetConnectorMass();
            ChVector3d inertia_connector = 2.0 * GetConnectorInertia();
            inertia_connector.y() += 2.0 * GetConnectorMass() * (0.5 * GetShoeWidth()) * (0.5 * GetShoeWidth());

            m_connector_L = chrono_types::make_shared<ChBody>();
            m_connector_L->SetName(m_name + "_connector");
            m_connector_L->SetTag(m_obj_tag);
            m_connector_L->SetPos(loc + (0.5 * GetShoeLength()) * xdir);
            m_connector_L->SetRot(rot);
            m_connector_L->SetMass(mass_connector);
            m_connector_L->SetInertiaXX(inertia_connector);
            chassis->GetSystem()->AddBody(m_connector_L);
        } break;
    }

    // Add contact geometry on shoe body
    m_geometry.CreateCollisionShapes(m_shoe, VehicleCollisionFamily::SHOE_FAMILY, sys->GetContactMethod());
    m_shoe->GetCollisionModel()->DisallowCollisionsWith(VehicleCollisionFamily::SHOE_FAMILY);

    // Create collision model for connector bodies
    // Note: even though the connector bodies only collide with the sprocket (through a custom collision detection
    // phase) and are not included in the Chrono collision detection, a collision model is still needed in order to
    // allow access back to the contactable (in this case the connector body) when processing contacts.
    m_connector_L->AddCollisionModel(chrono_types::make_shared<ChCollisionModel>());
    if (m_topology == DoublePinTrackShoeType::TWO_CONNECTORS)
        m_connector_R->AddCollisionModel(chrono_types::make_shared<ChCollisionModel>());
}

void ChTrackShoeDoublePin::Initialize(std::shared_ptr<ChChassis> chassis,
                                      const ChVector3d& loc_shoe,
                                      const ChQuaternion<>& rot_shoe,
                                      const ChVector3d& loc_connector_L,
                                      const ChVector3d& loc_connector_R,
                                      const ChQuaternion<>& rot_connector) {
    // Initialize at origin.
    ChTrackShoe::Initialize(chassis, VNULL, QUNIT);

    // Overwrite absolute body locations and orientations.
    auto chassis_body = chassis->GetBody();

    m_shoe->SetPos(chassis_body->TransformPointLocalToParent(loc_shoe));
    m_shoe->SetRot(chassis_body->GetRot() * rot_shoe);

    switch (m_topology) {
        case DoublePinTrackShoeType::TWO_CONNECTORS:
            m_connector_L->SetPos(chassis_body->TransformPointLocalToParent(loc_connector_L));
            m_connector_L->SetRot(chassis_body->GetRot() * rot_connector);

            m_connector_R->SetPos(chassis_body->TransformPointLocalToParent(loc_connector_R));
            m_connector_R->SetRot(chassis_body->GetRot() * rot_connector);
            break;
        case DoublePinTrackShoeType::ONE_CONNECTOR:
            m_connector_L->SetPos(chassis_body->TransformPointLocalToParent(0.5 * (loc_connector_L + loc_connector_R)));
            m_connector_L->SetRot(chassis_body->GetRot() * rot_connector);
            break;
    }
}

void ChTrackShoeDoublePin::InitializeInertiaProperties() {
    m_mass = GetShoeMass() + 2 * GetConnectorMass();
}

void ChTrackShoeDoublePin::UpdateInertiaProperties() {
    m_xform = m_shoe->GetFrameRefToAbs();

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;
    composite.AddComponent(m_shoe->GetFrameCOMToAbs(), m_shoe->GetMass(), m_shoe->GetInertia());
    switch (m_topology) {
        case DoublePinTrackShoeType::TWO_CONNECTORS:
            composite.AddComponent(m_connector_L->GetFrameCOMToAbs(), m_connector_L->GetMass(),
                                   m_connector_L->GetInertia());
            composite.AddComponent(m_connector_R->GetFrameCOMToAbs(), m_connector_R->GetMass(),
                                   m_connector_R->GetInertia());
            break;
        case DoublePinTrackShoeType::ONE_CONNECTOR:
            composite.AddComponent(m_connector_L->GetFrameCOMToAbs(), m_connector_L->GetMass(),
                                   m_connector_L->GetInertia());
            break;
    }

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(QUNIT);

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
}

double ChTrackShoeDoublePin::GetPitch() const {
    return GetShoeLength() + GetConnectorLength();
}

// -----------------------------------------------------------------------------
void ChTrackShoeDoublePin::AddVisualizationAssets(VisualizationType vis) {
    ChTrackShoeSegmented::AddVisualizationAssets(vis);
    switch (m_topology) {
        case DoublePinTrackShoeType::TWO_CONNECTORS:
            AddConnectorVisualization2(m_connector_L, vis);
            AddConnectorVisualization2(m_connector_R, vis);
            break;
        case DoublePinTrackShoeType::ONE_CONNECTOR:
            AddConnectorVisualization1(m_connector_L, vis);
            break;
    }
}

void ChTrackShoeDoublePin::RemoveVisualizationAssets() {
    switch (m_topology) {
        case DoublePinTrackShoeType::TWO_CONNECTORS:
            ChPart::RemoveVisualizationAssets(m_connector_L);
            ChPart::RemoveVisualizationAssets(m_connector_R);
            break;
        case DoublePinTrackShoeType::ONE_CONNECTOR:
            ChPart::RemoveVisualizationAssets(m_connector_L);
            break;
    }
    ChTrackShoeSegmented::RemoveVisualizationAssets();
}

void ChTrackShoeDoublePin::AddConnectorVisualization2(std::shared_ptr<ChBody> connector, VisualizationType vis) {
    assert(m_topology == DoublePinTrackShoeType::TWO_CONNECTORS);

    if (vis == VisualizationType::NONE)
        return;

    double c_length = GetConnectorLength();
    double c_width = GetConnectorWidth();
    double c_radius = GetConnectorRadius();

    utils::ChBodyGeometry::AddVisualizationCylinder(connector,                                       //
                                                    ChVector3d(-0.5 * c_length, -0.5 * c_width, 0),  //
                                                    ChVector3d(-0.5 * c_length, +0.5 * c_width, 0),  //
                                                    c_radius);

    utils::ChBodyGeometry::AddVisualizationCylinder(connector,                                      //
                                                    ChVector3d(0.5 * c_length, -0.5 * c_width, 0),  //
                                                    ChVector3d(0.5 * c_length, +0.5 * c_width, 0),  //
                                                    c_radius);

    auto box = chrono_types::make_shared<ChVisualShapeBox>(c_length, c_width, 2 * c_radius);
    connector->AddVisualShape(box, ChFrame<>());
}

void ChTrackShoeDoublePin::AddConnectorVisualization1(std::shared_ptr<ChBody> connector, VisualizationType vis) {
    assert(m_topology == DoublePinTrackShoeType::ONE_CONNECTOR);

    if (vis == VisualizationType::NONE)
        return;

    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetDiffuseColor(ChColor(1.0f, 0.68f, 0.2f));

    double offset = 0.5 * GetShoeWidth();
    double c_length = GetConnectorLength();
    double c_width = GetConnectorWidth();
    double c_radius = GetConnectorRadius();

    {
        utils::ChBodyGeometry::AddVisualizationCylinder(connector,                                                //
                                                        ChVector3d(-0.5 * c_length, +offset - 0.5 * c_width, 0),  //
                                                        ChVector3d(-0.5 * c_length, +offset + 0.5 * c_width, 0),  //
                                                        c_radius,                                                 //
                                                        mat);

        utils::ChBodyGeometry::AddVisualizationCylinder(connector,                                               //
                                                        ChVector3d(0.5 * c_length, +offset - 0.5 * c_width, 0),  //
                                                        ChVector3d(0.5 * c_length, +offset + 0.5 * c_width, 0),  //
                                                        c_radius,                                                //
                                                        mat);

        auto box = chrono_types::make_shared<ChVisualShapeBox>(c_length, c_width, 2 * c_radius);
        box->AddMaterial(mat);
        connector->AddVisualShape(box, ChFrame<>(ChVector3d(0, +offset, 0)));
    }

    {
        utils::ChBodyGeometry::AddVisualizationCylinder(connector,                                                //
                                                        ChVector3d(-0.5 * c_length, -offset - 0.5 * c_width, 0),  //
                                                        ChVector3d(-0.5 * c_length, -offset + 0.5 * c_width, 0),  //
                                                        c_radius,                                                 //
                                                        mat);

        utils::ChBodyGeometry::AddVisualizationCylinder(connector,                                               //
                                                        ChVector3d(0.5 * c_length, -offset - 0.5 * c_width, 0),  //
                                                        ChVector3d(0.5 * c_length, -offset + 0.5 * c_width, 0),  //
                                                        c_radius,                                                //
                                                        mat);

        auto box = chrono_types::make_shared<ChVisualShapeBox>(c_length, c_width, 2 * c_radius);
        box->AddMaterial(mat);
        connector->AddVisualShape(box, ChFrame<>(ChVector3d(0, -offset, 0)));
    }
}

// -----------------------------------------------------------------------------

void ChTrackShoeDoublePin::EnableTrackBendingStiffness(bool val) {
    switch (m_topology) {
        case DoublePinTrackShoeType::TWO_CONNECTORS:
            m_rsda_L->SetDisabled(val);
            m_rsda_R->SetDisabled(val);
            m_connection_rsda_L->SetDisabled(val);
            m_connection_rsda_R->SetDisabled(val);
            break;
        case DoublePinTrackShoeType::ONE_CONNECTOR:
            m_rsda_L->SetDisabled(val);
            m_connection_rsda_L->SetDisabled(val);
            break;
    }
}

void ChTrackShoeDoublePin::Connect(std::shared_ptr<ChTrackShoe> next,
                                   ChTrackAssembly* assembly,
                                   ChChassis* chassis,
                                   bool ccw) {
    switch (m_topology) {
        case DoublePinTrackShoeType::TWO_CONNECTORS:
            Connect2(next, assembly, chassis, ccw);
            break;
        case DoublePinTrackShoeType::ONE_CONNECTOR:
            Connect1(next, assembly, chassis, ccw);
            break;
    }
}

void ChTrackShoeDoublePin::Connect2(std::shared_ptr<ChTrackShoe> next,
                                    ChTrackAssembly* assembly,
                                    ChChassis* chassis,
                                    bool ccw) {
    assert(m_topology == DoublePinTrackShoeType::TWO_CONNECTORS);
    auto track = static_cast<ChTrackAssemblyDoublePin*>(assembly);
    assert(track->GetBushingData());

    ChSystem* system = m_shoe->GetSystem();
    double sign = ccw ? +1 : -1;

    ChVector3d pShoe_L;     // local point on shoe (left)
    ChVector3d pShoe_R;     // local point on shoe (right)
    ChVector3d pConnector;  // local point on connector

    ChVector3d loc_L;    // left point (expressed in absolute frame)
    ChVector3d loc_R;    // right point (expressed in absolute frame)
    ChQuaternion<> rot;  // orientation (expressed in absolute frame)

    // 1. Connections between this shoe body and connector bodies

    // Create and initialize the revolute joints/bushings between shoe body and connector bodies.
    pShoe_L = ChVector3d(sign * GetShoeLength() / 2, +GetShoeWidth() / 2, 0);
    pShoe_R = ChVector3d(sign * GetShoeLength() / 2, -GetShoeWidth() / 2, 0);
    pConnector = ChVector3d(-sign * GetConnectorLength() / 2, 0, 0);

    loc_L = m_shoe->TransformPointLocalToParent(pShoe_L);
    loc_R = m_shoe->TransformPointLocalToParent(pShoe_R);
    rot = m_shoe->GetRot() * QuatFromAngleX(CH_PI_2);

    m_joint_L =
        chrono_types::make_shared<ChJoint>(ChJoint::Type::REVOLUTE, m_name + "_pin_L", m_connector_L,
                                                  m_shoe, ChFrame<>(loc_L, rot), track->GetBushingData());
    m_joint_L->SetTag(m_obj_tag);
    chassis->AddJoint(m_joint_L);

    m_joint_R =
        chrono_types::make_shared<ChJoint>(ChJoint::Type::REVOLUTE, m_name + "_pin_R", m_connector_R,
                                                  m_shoe, ChFrame<>(loc_R, rot), track->GetBushingData());
    m_joint_R->SetTag(m_obj_tag);
    chassis->AddJoint(m_joint_R);

    // Optionally, include rotational spring-dampers to model track bending stiffness.
    // The RSDA frames are aligned with the corresponding body frames and the springs have a default zero rest angle.
    if (track->GetTorqueFunctor()) {
        ChQuaternion<> z2y = QuatFromAngleX(-CH_PI_2);

        m_rsda_L = chrono_types::make_shared<ChLinkRSDA>();
        m_rsda_L->SetName(m_name + "_rsda_pin_L");
        m_rsda_L->SetTag(m_obj_tag);
        m_rsda_L->Initialize(m_shoe, m_connector_L, true, ChFrame<>(pShoe_L, z2y), ChFrame<>(pConnector, z2y));
        m_rsda_L->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(m_rsda_L);

        m_rsda_R = chrono_types::make_shared<ChLinkRSDA>();
        m_rsda_R->SetName(m_name + "_rsda_pin_R");
        m_rsda_R->SetTag(m_obj_tag);
        m_rsda_R->Initialize(m_shoe, m_connector_R, true, ChFrame<>(pShoe_R, z2y), ChFrame<>(pConnector, z2y));
        m_rsda_R->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(m_rsda_R);
    }

    // 2. Connections between connector bodies and next shoe body

    pShoe_L = ChVector3d(-sign * GetShoeLength() / 2, +GetShoeWidth() / 2, 0);
    pShoe_R = ChVector3d(-sign * GetShoeLength() / 2, -GetShoeWidth() / 2, 0);
    pConnector = ChVector3d(sign * GetConnectorLength() / 2, 0, 0);

    loc_L = m_connector_L->TransformPointLocalToParent(pConnector);
    loc_R = m_connector_R->TransformPointLocalToParent(pConnector);

    // Create and initialize revolute bushings between connector bodies and next shoe body.
    rot = m_connector_L->GetRot() * QuatFromAngleX(CH_PI_2);
    m_connection_joint_L = chrono_types::make_shared<ChJoint>(ChJoint::Type::REVOLUTE, m_name + "_cpin_L",
                                                                     next->GetShoeBody(), m_connector_L,
                                                                     ChFrame<>(loc_L, rot), track->GetBushingData());
    m_connection_joint_L->SetTag(m_obj_tag);
    chassis->AddJoint(m_connection_joint_L);

    rot = m_connector_R->GetRot() * QuatFromAngleX(CH_PI_2);
    m_connection_joint_R = chrono_types::make_shared<ChJoint>(ChJoint::Type::REVOLUTE, m_name + "_cpin_R",
                                                                     next->GetShoeBody(), m_connector_R,
                                                                     ChFrame<>(loc_R, rot), track->GetBushingData());
    m_connection_joint_R->SetTag(m_obj_tag);
    chassis->AddJoint(m_connection_joint_R);

    // Optionally, include rotational spring-dampers to model track bending stiffness
    // The RSDA frames are aligned with the corresponding body frames and the springs have a default zero rest angle.
    if (track->GetTorqueFunctor()) {
        ChQuaternion<> z2y = QuatFromAngleX(-CH_PI_2);

        m_connection_rsda_L = chrono_types::make_shared<ChLinkRSDA>();
        m_connection_rsda_L->SetName(m_name + "_rsda_cpin_L");
        m_connection_rsda_L->SetTag(m_obj_tag);
        m_connection_rsda_L->Initialize(m_connector_L, next->GetShoeBody(), true, ChFrame<>(pConnector, z2y),
                                        ChFrame<>(pShoe_L, z2y));
        m_connection_rsda_L->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(m_connection_rsda_L);

        m_connection_rsda_R = chrono_types::make_shared<ChLinkRSDA>();
        m_connection_rsda_R->SetName(m_name + "_rsda_cpin_R");
        m_connection_rsda_R->SetTag(m_obj_tag);
        m_connection_rsda_R->Initialize(m_connector_R, next->GetShoeBody(), true, ChFrame<>(pConnector, z2y),
                                        ChFrame<>(pShoe_R, z2y));
        m_connection_rsda_R->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(m_connection_rsda_R);
    }
}

void ChTrackShoeDoublePin::Connect1(std::shared_ptr<ChTrackShoe> next,
                                    ChTrackAssembly* assembly,
                                    ChChassis* chassis,
                                    bool ccw) {
    assert(m_topology == DoublePinTrackShoeType::ONE_CONNECTOR);

    auto track = static_cast<ChTrackAssemblyDoublePin*>(assembly);
    ChSystem* system = m_shoe->GetSystem();
    double sign = ccw ? +1 : -1;

    ChVector3d pShoe;       // local point on shoe
    ChVector3d pConnector;  // local point on connector

    ChVector3d loc;      // point (expressed in absolute frame)
    ChQuaternion<> rot;  // orientation (expressed in absolute frame)

    // 1. Connections between this shoe body and connector body

    // Create and initialize the joint/bushing between shoe body and connector body.
    pShoe = ChVector3d(sign * GetShoeLength() / 2, 0, 0);
    pConnector = ChVector3d(-sign * GetConnectorLength() / 2, 0, 0);

    loc = m_shoe->TransformPointLocalToParent(pShoe);

    if (track->GetBushingData() || m_index != 0) {
        rot = m_shoe->GetRot() * QuatFromAngleX(CH_PI_2);
        m_joint_L =
            chrono_types::make_shared<ChJoint>(ChJoint::Type::REVOLUTE, m_name + "_pin", m_connector_L,
                                                      m_shoe, ChFrame<>(loc, rot), track->GetBushingData());
        m_joint_L->SetTag(m_obj_tag);
        chassis->AddJoint(m_joint_L);
    } else {
        m_joint_L = chrono_types::make_shared<ChJoint>(ChJoint::Type::SPHERICAL, m_name + "_sph",
                                                              m_connector_L, m_shoe, ChFrame<>(loc, QUNIT));
        m_joint_L->SetTag(m_obj_tag);
        chassis->AddJoint(m_joint_L);
    }

    // Optionally, include rotational spring-damper to model track bending stiffness.
    // The RSDA frame is aligned with the corresponding body frame and the spring has a default zero rest angle.
    if (track->GetTorqueFunctor()) {
        ChQuaternion<> z2y = QuatFromAngleX(-CH_PI_2);

        m_rsda_L = chrono_types::make_shared<ChLinkRSDA>();
        m_rsda_L->SetName(m_name + "_rsda_pin");
        m_rsda_L->SetTag(m_obj_tag);
        m_rsda_L->Initialize(m_shoe, m_connector_L, true, ChFrame<>(pShoe, z2y), ChFrame<>(pConnector, z2y));
        m_rsda_L->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(m_rsda_L);
    }

    // 2. Connection between connector body and next shoe body

    pShoe = ChVector3d(-sign * GetShoeLength() / 2, 0, 0);
    pConnector = ChVector3d(sign * GetConnectorLength() / 2, 0, 0);

    loc = m_connector_L->TransformPointLocalToParent(pConnector);

    if (track->GetBushingData() || m_index != 0) {
        rot = m_connector_L->GetRot() * QuatFromAngleX(CH_PI_2);
        m_connection_joint_L = chrono_types::make_shared<ChJoint>(
            ChJoint::Type::REVOLUTE, m_name + "_cpin", next->GetShoeBody(), m_connector_L, ChFrame<>(loc, rot),
            track->GetBushingData());
        m_connection_joint_L->SetTag(m_obj_tag);
        chassis->AddJoint(m_connection_joint_L);
    } else {
        rot = m_connector_L->GetRot() * QuatFromAngleY(-CH_PI_2);
        m_connection_joint_L =
            chrono_types::make_shared<ChJoint>(ChJoint::Type::UNIVERSAL, m_name + "_cuniv",
                                                      next->GetShoeBody(), m_connector_L, ChFrame<>(loc, rot));
        m_connection_joint_L->SetTag(m_obj_tag);
        chassis->AddJoint(m_connection_joint_L);
    }

    // Optionally, include rotational spring-dampers to model track bending stiffness
    // The RSDA frame is aligned with the corresponding body frame and the spring has a default zero rest angle.
    if (track->GetTorqueFunctor()) {
        ChQuaternion<> z2y = QuatFromAngleX(-CH_PI_2);

        m_connection_rsda_L = chrono_types::make_shared<ChLinkRSDA>();
        m_connection_rsda_L->SetName(m_name + "_rsda_cpin_L");
        m_connection_rsda_L->SetTag(m_obj_tag);
        m_connection_rsda_L->Initialize(m_connector_L, next->GetShoeBody(), true, ChFrame<>(pConnector, z2y),
                                        ChFrame<>(pShoe, z2y));
        m_connection_rsda_L->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(m_connection_rsda_L);
    }
}

ChVector3d ChTrackShoeDoublePin::GetTension() const {
    switch (m_topology) {
        case DoublePinTrackShoeType::TWO_CONNECTORS:
            return m_joint_L->GetForce() + m_joint_R->GetForce();
        case DoublePinTrackShoeType::ONE_CONNECTOR:
            return m_joint_L->GetForce();
    }
    return ChVector3d(0);
}

// -----------------------------------------------------------------------------
void ChTrackShoeDoublePin::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;

    bodies.push_back(m_shoe);

    switch (m_topology) {
        case DoublePinTrackShoeType::TWO_CONNECTORS:
            bodies.push_back(m_connector_L);
            bodies.push_back(m_connector_R);

            m_joint_L->IsKinematic() ? joints.push_back(m_joint_L->GetAsLink())
                                     : bushings.push_back(m_joint_L->GetAsBushing());
            m_joint_R->IsKinematic() ? joints.push_back(m_joint_R->GetAsLink())
                                     : bushings.push_back(m_joint_R->GetAsBushing());
            m_connection_joint_L->IsKinematic() ? joints.push_back(m_connection_joint_L->GetAsLink())
                                                : bushings.push_back(m_connection_joint_L->GetAsBushing());
            m_connection_joint_R->IsKinematic() ? joints.push_back(m_connection_joint_R->GetAsLink())
                                                : bushings.push_back(m_connection_joint_R->GetAsBushing());

            break;
        case DoublePinTrackShoeType::ONE_CONNECTOR:
            bodies.push_back(m_connector_L);

            m_joint_L->IsKinematic() ? joints.push_back(m_joint_L->GetAsLink())
                                     : bushings.push_back(m_joint_L->GetAsBushing());
            m_connection_joint_L->IsKinematic() ? joints.push_back(m_connection_joint_L->GetAsLink())
                                                : bushings.push_back(m_connection_joint_L->GetAsBushing());

            break;
    }

    ExportBodyList(jsonDocument, bodies);
    ExportJointList(jsonDocument, joints);
    ExportBodyLoadList(jsonDocument, bushings);
}

void ChTrackShoeDoublePin::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;

    bodies.push_back(m_shoe);

    switch (m_topology) {
        case DoublePinTrackShoeType::TWO_CONNECTORS:
            bodies.push_back(m_connector_L);
            bodies.push_back(m_connector_R);

            m_joint_L->IsKinematic() ? joints.push_back(m_joint_L->GetAsLink())
                                     : bushings.push_back(m_joint_L->GetAsBushing());
            m_joint_R->IsKinematic() ? joints.push_back(m_joint_R->GetAsLink())
                                     : bushings.push_back(m_joint_R->GetAsBushing());
            m_connection_joint_L->IsKinematic() ? joints.push_back(m_connection_joint_L->GetAsLink())
                                                : bushings.push_back(m_connection_joint_L->GetAsBushing());
            m_connection_joint_R->IsKinematic() ? joints.push_back(m_connection_joint_R->GetAsLink())
                                                : bushings.push_back(m_connection_joint_R->GetAsBushing());

            break;
        case DoublePinTrackShoeType::ONE_CONNECTOR:
            bodies.push_back(m_connector_L);

            m_joint_L->IsKinematic() ? joints.push_back(m_joint_L->GetAsLink())
                                     : bushings.push_back(m_joint_L->GetAsBushing());
            m_connection_joint_L->IsKinematic() ? joints.push_back(m_connection_joint_L->GetAsLink())
                                                : bushings.push_back(m_connection_joint_L->GetAsBushing());

            break;
    }

    database.WriteBodies(bodies);
    database.WriteJoints(joints);
    database.WriteBodyLoads(bushings);
}

}  // end namespace vehicle
}  // end namespace chrono
