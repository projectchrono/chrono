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
// Simple brake created with constant torque opposing wheel rotation.
// It just uses a speed-dependant torque, so it fits in ODEs because it does not
// use NSC set valued constraints (the  drawback is that it cannot simulate
// sticking brakes).
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

namespace chrono {
namespace vehicle {

ChBrakeSimple::ChBrakeSimple(const std::string& name) : ChBrake(name), m_modulation(0) {
    m_brake = std::make_shared<ChLinkBrake>();
}

void ChBrakeSimple::Initialize(std::shared_ptr<ChLinkLockRevolute> hub) {
    ChSystem* my_system = hub->GetSystem();

    // Reuse the same bodies and link coordinate of the hub revolute joint
    // Note that we wrap raw pointers in local shared_ptr.  For this, we must provide
    // custom empty deleters (to prevent deleting the objects when the local shared_ptr
    // go out of scope).
    std::shared_ptr<ChBodyFrame> mbf1(hub->GetBody1(), [](ChBodyFrame*){});
    std::shared_ptr<ChBodyFrame> mbf2(hub->GetBody2(), [](ChBodyFrame*){});

    // Downcast to ChBody shared_ptr
    auto mb1 = std::dynamic_pointer_cast<ChBody>(mbf1);
    auto mb2 = std::dynamic_pointer_cast<ChBody>(mbf2);

    m_brake->Initialize(mb1, mb2, true, hub->GetMarker1()->GetCoord(), hub->GetMarker2()->GetCoord());
    my_system->AddLink(m_brake);
}

void ChBrakeSimple::Synchronize(double modulation) {
    m_modulation = modulation;
    m_brake->Set_brake_torque(modulation * GetMaxBrakingTorque());
}

}  // end namespace vehicle
}  // end namespace chrono
