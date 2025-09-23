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
// Base class for a single-pin track shoe (template definition).
// A single-pin track shoe can be either of CENTRAL_PIN or LATERAL_PIN type.
//
// =============================================================================

#include "chrono/core/ChGlobal.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblySinglePin.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSinglePin.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChTrackShoeSinglePin::ChTrackShoeSinglePin(const std::string& name) : ChTrackShoeSegmented(name) {}

ChTrackShoeSinglePin::~ChTrackShoeSinglePin() {
    if (!m_shoe)
        return;

    auto sys = m_shoe->GetSystem();
    if (sys) {
        ChChassis::RemoveJoint(m_joint);
        if (m_rsda)
            sys->Remove(m_rsda);
    }
}

// -----------------------------------------------------------------------------
void ChTrackShoeSinglePin::Construct(std::shared_ptr<ChChassis> chassis,
                                     const ChVector3d& location,
                                     const ChQuaternion<>& rotation) {
    ChTrackShoeSegmented::Construct(chassis, location, rotation);

    ChSystem* sys = chassis->GetSystem();

    // Create the shoe body.
    auto chassis_body = chassis->GetBody();

    ChVector3d loc = chassis_body->TransformPointLocalToParent(location);
    ChQuaternion<> rot = chassis_body->GetRot() * rotation;
    m_shoe = chrono_types::make_shared<ChBody>();
    m_shoe->SetName(m_name + "_shoe");
    m_shoe->SetTag(m_obj_tag);
    m_shoe->SetPos(loc);
    m_shoe->SetRot(rot);
    m_shoe->SetMass(GetShoeMass());
    m_shoe->SetInertiaXX(GetShoeInertia());
    m_shoe->EnableCollision(true);
    chassis->GetSystem()->AddBody(m_shoe);

    // Add contact geometry on shoe body
    m_geometry.CreateCollisionShapes(m_shoe, VehicleCollisionFamily::SHOE_FAMILY, sys->GetContactMethod());
    m_shoe->GetCollisionModel()->DisallowCollisionsWith(VehicleCollisionFamily::SHOE_FAMILY);
}

void ChTrackShoeSinglePin::InitializeInertiaProperties() {
    m_mass = GetShoeMass();
    m_inertia = ChMatrix33<>(0);
    m_inertia.diagonal() = GetShoeInertia().eigen();
    m_com = ChFrame<>();
}

void ChTrackShoeSinglePin::UpdateInertiaProperties() {
    m_xform = m_shoe->GetFrameRefToAbs();
}

// -----------------------------------------------------------------------------

void ChTrackShoeSinglePin::EnableTrackBendingStiffness(bool val) {
    m_rsda->SetDisabled(val);
}

void ChTrackShoeSinglePin::Connect(std::shared_ptr<ChTrackShoe> next,
                                   ChTrackAssembly* assembly,
                                   ChChassis* chassis,
                                   bool ccw) {
    auto track = static_cast<ChTrackAssemblySinglePin*>(assembly);
    ChSystem* system = m_shoe->GetSystem();
    double sign = ccw ? +1 : -1;

    ChVector3d p_shoe = ChVector3d(+sign * GetPitch() / 2, 0, 0);  // local point on this shoe
    ChVector3d p_next = ChVector3d(-sign * GetPitch() / 2, 0, 0);  // local point on next shoe
    ChVector3d loc = m_shoe->TransformPointLocalToParent(p_shoe);  // connection point (expressed in absolute frame)

    if (track->GetBushingData() || (m_index != 0 && m_index != 1)) {
        // Create and initialize the revolute joint (rotation axis along Z)
        auto rot = m_shoe->GetRot() * QuatFromAngleX(CH_PI_2);
        m_joint = chrono_types::make_shared<ChJoint>(ChJoint::Type::REVOLUTE, m_name + "_pin",
                                                            next->GetShoeBody(), m_shoe, ChFrame<>(loc, rot),
                                                            track->GetBushingData());
        m_joint->SetTag(m_obj_tag);
        chassis->AddJoint(m_joint);
    } else if (m_index == 0) {
        m_joint = chrono_types::make_shared<ChJoint>(ChJoint::Type::SPHERICAL, m_name + "_sph",
                                                            next->GetShoeBody(), m_shoe, ChFrame<>(loc, QUNIT));
        chassis->AddJoint(m_joint);
    } else if (m_index == 1) {
        auto rot = m_shoe->GetRot() * QuatFromAngleY(-CH_PI_2);
        m_joint = chrono_types::make_shared<ChJoint>(ChJoint::Type::UNIVERSAL, m_name + "_univ",
                                                            next->GetShoeBody(), m_shoe, ChFrame<>(loc, rot));
        m_joint->SetTag(m_obj_tag);
        chassis->AddJoint(m_joint);
    }

    // Optionally, include rotational spring-damper to model track bending stiffness
    // The RSDA frames are aligned with the corresponding body frames and the spring has a default zero rest angle.
    if (track->GetTorqueFunctor()) {
        ChQuaternion<> z2y = QuatFromAngleX(-CH_PI_2);

        m_rsda = chrono_types::make_shared<ChLinkRSDA>();
        m_rsda->SetName(m_name + "_rsda");
        m_rsda->SetTag(m_obj_tag);
        m_rsda->Initialize(m_shoe, next->GetShoeBody(), true, ChFrame<>(p_shoe, z2y), ChFrame<>(p_next, z2y));
        m_rsda->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(m_rsda);
    }
}

ChVector3d ChTrackShoeSinglePin::GetTension() const {
    return m_joint->GetForce();
}

// -----------------------------------------------------------------------------
void ChTrackShoeSinglePin::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_shoe);
    ExportBodyList(jsonDocument, bodies);
}

void ChTrackShoeSinglePin::Output(ChOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_shoe);
    database.WriteBodies(bodies);
}

}  // end namespace vehicle
}  // end namespace chrono
