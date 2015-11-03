// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#include "chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeSimple.h"

namespace chrono {
namespace vehicle {

ChTrackBrakeSimple::ChTrackBrakeSimple() : m_braking(0) {
    m_brake = ChSharedPtr<ChLinkBrake>(new ChLinkBrake);
}

void ChTrackBrakeSimple::Initialize(ChSharedPtr<ChLinkLockRevolute> hub) {
    ChSystem* my_system = hub->GetSystem();

    // Reuse the same bodies and link coordinate of the hub revolute joint...
    ChSharedPtr<ChBodyFrame> mbf1(hub->GetBody1());
    hub->GetBody1()->AddRef();  // because mbf1(mhub->GetBody1()) got a plain pointer, so transformed to shared
    ChSharedPtr<ChBodyFrame> mbf2(hub->GetBody2());
    hub->GetBody2()->AddRef();  // because mbf2(mhub->GetBody2()) got a plain pointer, so transformed to shared
    ChSharedPtr<ChBody> mb1 = mbf1.DynamicCastTo<ChBody>();
    ChSharedPtr<ChBody> mb2 = mbf2.DynamicCastTo<ChBody>();

    m_brake->Initialize(mb1, mb2, hub->GetMarker2()->GetCoord());
    my_system->AddLink(m_brake);
}

void ChTrackBrakeSimple::Update(double braking) {
    m_braking = braking;
    m_brake->Set_brake_torque(braking * GetMaxBrakingTorque());
}

}  // end namespace vehicle
}  // end namespace chrono
