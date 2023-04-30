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
// Simple brake created with constant torque opposing sprocket rotation.
// It just uses a speed-dependent torque.
//
// =============================================================================

#include "chrono/physics/ChLinkLock.h"

#include "chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeSimple.h"

namespace chrono {
namespace vehicle {

ChTrackBrakeSimple::ChTrackBrakeSimple(const std::string& name) : ChTrackBrake(name), m_braking(0) {
    m_brake = chrono_types::make_shared<ChLinkBrake>();
}

ChTrackBrakeSimple::~ChTrackBrakeSimple() {
    auto sys = m_brake->GetSystem();
    if (sys) {
        sys->Remove(m_brake);
    }
}

void ChTrackBrakeSimple::Initialize(std::shared_ptr<ChChassis> chassis, std::shared_ptr<ChSprocket> sprocket) {
    ChTrackBrake::Initialize(chassis, sprocket);

    auto hub = sprocket->GetRevolute();
    auto sys = hub->GetSystem();

    // Reuse the same bodies and link coordinate of the hub revolute joint.
    // Note that we wrap raw pointers in local shared_ptr.  For this, we must provide
    // custom empty deleters (to prevent deleting the objects when the local shared_ptr
    // go out of scope).
    std::shared_ptr<ChBodyFrame> mbf1(hub->GetBody1(), [](ChBodyFrame*){});
    std::shared_ptr<ChBodyFrame> mbf2(hub->GetBody2(), [](ChBodyFrame*){});

    // Downcast to ChBody shared_ptr
    auto mb1 = std::dynamic_pointer_cast<ChBody>(mbf1);
    auto mb2 = std::dynamic_pointer_cast<ChBody>(mbf2);

    m_brake->Initialize(mb1, mb2, true, hub->GetMarker1()->GetCoord(), hub->GetMarker2()->GetCoord());
    sys->AddLink(m_brake);
}

void ChTrackBrakeSimple::Synchronize(double braking) {
    m_braking = braking;
    m_brake->Set_brake_torque(braking * GetMaxBrakingTorque());
}

}  // end namespace vehicle
}  // end namespace chrono
