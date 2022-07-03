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
// Base class for a track wheel (road wheel, idler wheel, or roller wheel).
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChTrackWheel.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackWheel::ChTrackWheel(const std::string& name) : ChPart(name), m_track(nullptr) {}

ChTrackWheel::~ChTrackWheel() {
    auto sys = m_wheel->GetSystem();
    if (sys) {
        sys->Remove(m_wheel);
        sys->Remove(m_revolute);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackWheel::Initialize(std::shared_ptr<ChChassis> chassis,
                              std::shared_ptr<ChBody> carrier,
                              const ChVector<>& location,
                              ChTrackAssembly* track) {
    m_track = track;

    // Express the wheel reference frame in the absolute coordinate system.
    ChFrame<> wheel_to_abs(location);
    wheel_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrame_REF_to_abs());

    // Create and initialize the wheel body.
    m_wheel = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_wheel->SetNameString(m_name + "_wheel");
    m_wheel->SetIdentifier(BodyID::WHEEL_BODY);
    m_wheel->SetPos(wheel_to_abs.GetPos());
    m_wheel->SetRot(wheel_to_abs.GetRot());
    m_wheel->SetMass(GetMass());
    m_wheel->SetInertiaXX(GetInertia());
    chassis->GetSystem()->AddBody(m_wheel);

    // Create and initialize the revolute joint between wheel and carrier.
    // The axis of rotation is the y axis of the wheel reference frame.
    m_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(carrier, m_wheel,
                           ChCoordsys<>(wheel_to_abs.GetPos(), wheel_to_abs.GetRot() * Q_from_AngX(CH_C_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute);
}

void ChTrackWheel::InitializeInertiaProperties() {
    m_mass = GetMass();
    m_inertia = ChMatrix33<>(0);
    m_inertia.diagonal() = GetInertia().eigen();
    m_com = ChFrame<>();
}

void ChTrackWheel::UpdateInertiaProperties() {
    m_xform = m_wheel->GetFrame_REF_to_abs();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackWheel::LogConstraintViolations() {
    ChVectorDynamic<> C = m_revolute->GetConstraintViolation();
    GetLog() << "  Road-wheel revolute\n";
    GetLog() << "  " << C(0) << "  ";
    GetLog() << "  " << C(1) << "  ";
    GetLog() << "  " << C(2) << "  ";
    GetLog() << "  " << C(3) << "  ";
    GetLog() << "  " << C(4) << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackWheel::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_wheel);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    ChPart::ExportJointList(jsonDocument, joints);
}

void ChTrackWheel::Output(ChVehicleOutput& database) const {
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
