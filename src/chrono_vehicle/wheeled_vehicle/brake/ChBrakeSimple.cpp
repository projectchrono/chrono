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
// Authors: Alessandro Tasora
// =============================================================================
//
// Simple brake created with constant torque opposing wheel rotation.
// It just uses a speed-dependent torque, so it fits in ODEs because it does not
// use NSC set valued constraints (the  drawback is that it cannot simulate
// sticking brakes).
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

namespace chrono {
namespace vehicle {

ChBrakeSimple::ChBrakeSimple(const std::string& name)
    : ChBrake(name), m_modulation(0), m_locked(false) {
    m_brake = chrono_types::make_shared<ChLinkBrake>();
}

ChBrakeSimple::~ChBrakeSimple() {
    auto sys = m_brake->GetSystem();
    if (sys) {
        sys->Remove(m_brake);
    }
}

void ChBrakeSimple::Initialize(std::shared_ptr<ChChassis> chassis,
                               std::shared_ptr<ChSuspension> suspension,
                               VehicleSide side) {
    ChBrake::Initialize(chassis, suspension, side);

    m_hub = suspension->GetRevolute(side);

    // Reuse the same bodies and link coordinate of the hub revolute joint
    // Note that we wrap raw pointers in local shared_ptr.  For this, we must provide
    // custom empty deleters (to prevent deleting the objects when the local shared_ptr
    // go out of scope).
    std::shared_ptr<ChBodyFrame> mbf1(m_hub->GetBody1(), [](ChBodyFrame*) {});
    std::shared_ptr<ChBodyFrame> mbf2(m_hub->GetBody2(), [](ChBodyFrame*) {});

    // Downcast to ChBody shared_ptr
    auto mb1 = std::dynamic_pointer_cast<ChBody>(mbf1);
    auto mb2 = std::dynamic_pointer_cast<ChBody>(mbf2);

    m_brake->Initialize(mb1, mb2, true, m_hub->GetMarker1()->GetCoord(), m_hub->GetMarker2()->GetCoord());
    m_hub->GetSystem()->AddLink(m_brake);
}

void ChBrakeSimple::Synchronize(double modulation) {
    m_modulation = modulation;
    m_brake->Set_brake_torque(modulation * GetMaxBrakingTorque());
    
    // If braking input is large enough, lock the brake
    if (!m_can_lock)
        return;

    if (modulation > 0.99 && !m_locked) {
        m_hub->Lock(true);
        m_locked = true;
    } else if (modulation <= 0.99 && m_locked) {
        m_hub->Lock(false);
        m_locked = false;
    }
}

}  // end namespace vehicle
}  // end namespace chrono
