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
// Base class for a tracked vehicle roller.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChRoller.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChRoller::ChRoller(const std::string& name) : ChPart(name), m_track(nullptr) {}

ChRoller::~ChRoller() {
    auto sys = m_wheel->GetSystem();
    if (sys) {
        sys->Remove(m_wheel);
        sys->Remove(m_revolute);
    }
}

// -----------------------------------------------------------------------------
void ChRoller::Initialize(std::shared_ptr<ChChassis> chassis, const ChVector<>& location, ChTrackAssembly* track) {
    m_parent = chassis;
    m_rel_loc = location;
    m_track = track;

    // Express the roller reference frame in the absolute coordinate system.
    ChFrame<> roller_to_abs(location);
    roller_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrame_REF_to_abs());

    // Create and initialize the roller body.
    m_wheel = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_wheel->SetNameString(m_name + "_wheel");
    m_wheel->SetIdentifier(BodyID::ROLER_BODY);
    m_wheel->SetPos(roller_to_abs.GetPos());
    m_wheel->SetRot(roller_to_abs.GetRot());
    m_wheel->SetMass(GetRollerMass());
    m_wheel->SetInertiaXX(GetRollerInertia());
    chassis->GetSystem()->AddBody(m_wheel);

    // Create and initialize the revolute joint between roller and chassis.
    // The axis of rotation is the y axis of the road wheel reference frame.
    m_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(chassis->GetBody(), m_wheel,
                           ChCoordsys<>(roller_to_abs.GetPos(), roller_to_abs.GetRot() * Q_from_AngX(CH_C_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute);
}

void ChRoller::InitializeInertiaProperties() {
    m_mass = GetRollerMass();
    m_inertia = ChMatrix33<>(0);
    m_inertia.diagonal() = GetRollerInertia().eigen();
    m_com = ChFrame<>();
}

void ChRoller::UpdateInertiaProperties() {
    m_xform = m_wheel->GetFrame_REF_to_abs();
}

// -----------------------------------------------------------------------------
void ChRoller::LogConstraintViolations() {
    ChVectorDynamic<> C = m_revolute->GetConstraintViolation();
    GetLog() << "  Road-wheel revolute\n";
    GetLog() << "  " << C(0) << "  ";
    GetLog() << "  " << C(1) << "  ";
    GetLog() << "  " << C(2) << "  ";
    GetLog() << "  " << C(3) << "  ";
    GetLog() << "  " << C(4) << "\n";
}

// -----------------------------------------------------------------------------
void ChRoller::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_wheel);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    ChPart::ExportJointList(jsonDocument, joints);
}

void ChRoller::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_wheel);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    database.WriteJoints(joints);
}

}  // end namespace vehicle
}  // end namespace chrono
