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
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChBoxShape.h"
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
        ChChassis::RemoveJoint(m_connection_joint);
        if (m_connection_rsda)
            sys->Remove(m_connection_rsda);
    }
}

// -----------------------------------------------------------------------------
void ChTrackShoeSinglePin::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                      const ChVector<>& location,
                                      const ChQuaternion<>& rotation) {
    ChSystem* sys = chassis->GetSystem();

    // Create the shoe body.
    ChVector<> loc = chassis->TransformPointLocalToParent(location);
    ChQuaternion<> rot = chassis->GetRot() * rotation;
    m_shoe = std::shared_ptr<ChBody>(sys->NewBody());
    m_shoe->SetNameString(m_name + "_shoe");
    m_shoe->SetIdentifier(BodyID::SHOE_BODY);
    m_shoe->SetPos(loc);
    m_shoe->SetRot(rot);
    m_shoe->SetMass(GetShoeMass());
    m_shoe->SetInertiaXX(GetShoeInertia());
    m_shoe->SetCollide(true);
    chassis->GetSystem()->AddBody(m_shoe);

    // Create all contact materials
    CreateContactMaterials(sys->GetContactMethod());

    // Add contact geometry on shoe body
    m_geometry.AddCollisionShapes(m_shoe, TrackedCollisionFamily::SHOES);
    m_shoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::SHOES);
}

void ChTrackShoeSinglePin::InitializeInertiaProperties() {
    m_mass = GetShoeMass();
    m_inertia = ChMatrix33<>(0);
    m_inertia.diagonal() = GetShoeInertia().eigen();
    m_com = ChFrame<>();
}

void ChTrackShoeSinglePin::UpdateInertiaProperties() {
    m_xform = m_shoe->GetFrame_REF_to_abs();
}

// -----------------------------------------------------------------------------

void ChTrackShoeSinglePin::EnableTrackBendingStiffness(bool val) {
    m_connection_rsda->SetDisabled(val);
}

void ChTrackShoeSinglePin::Connect(std::shared_ptr<ChTrackShoe> next,
                                   ChTrackAssembly* assembly,
                                   ChChassis* chassis,
                                   bool ccw) {
    auto track = static_cast<ChTrackAssemblySinglePin*>(assembly);
    ChSystem* system = m_shoe->GetSystem();
    double sign = ccw ? +1 : -1;

    ChVector<> p_shoe = ChVector<>(+sign * GetPitch() / 2, 0, 0);  // local point on this shoe
    ChVector<> p_next = ChVector<>(-sign * GetPitch() / 2, 0, 0);  // local point on next shoe
    ChVector<> loc = m_shoe->TransformPointLocalToParent(p_shoe);  // connection point (expressed in absolute frame)

    if (m_index == 0 && !track->GetBushingData()) {
        // Create and initialize a point-line joint (sliding line along X)
        auto rot = m_shoe->GetRot() * Q_from_AngZ(CH_C_PI_2);
        auto pointline =
            chrono_types::make_shared<ChVehicleJoint>(ChVehicleJoint::Type::POINTLINE, m_name + "_pointline", m_shoe,
                                                      next->GetShoeBody(), ChCoordsys<>(loc, rot));
        chassis->AddJoint(pointline);
        m_connection_joint = pointline;
    } else {
        // Create and initialize the revolute joint (rotation axis along Z)
        auto rot = m_shoe->GetRot() * Q_from_AngX(CH_C_PI_2);
        auto revolute = chrono_types::make_shared<ChVehicleJoint>(ChVehicleJoint::Type::REVOLUTE, m_name + "_revolute",
                                                                  m_shoe, next->GetShoeBody(), ChCoordsys<>(loc, rot),
                                                                  track->GetBushingData());
        chassis->AddJoint(revolute);
        m_connection_joint = revolute;
    }

    // Optionally, include rotational spring-damper to model track bending stiffness
    // The RSDA frames are aligned with the corresponding body frames and the spring has a default zero rest angle.
    if (track->GetTorqueFunctor()) {
        ChQuaternion<> z2y = Q_from_AngX(-CH_C_PI_2);

        m_connection_rsda = chrono_types::make_shared<ChLinkRSDA>();
        m_connection_rsda->SetNameString(m_name + "_rsda");
        m_connection_rsda->Initialize(m_shoe, next->GetShoeBody(), true, ChCoordsys<>(p_shoe, z2y),
                                      ChCoordsys<>(p_next, z2y));
        m_connection_rsda->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(m_connection_rsda);
    }
}

// -----------------------------------------------------------------------------
void ChTrackShoeSinglePin::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_shoe);
    ChPart::ExportBodyList(jsonDocument, bodies);
}

void ChTrackShoeSinglePin::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_shoe);
    database.WriteBodies(bodies);
}

}  // end namespace vehicle
}  // end namespace chrono
