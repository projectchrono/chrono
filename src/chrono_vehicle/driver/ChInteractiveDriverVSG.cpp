// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// VSG-based GUI driver for the a vehicle. This class implements the
// functionality required by its base ChDriver class using keyboard or joystick
// inputs.
// =============================================================================

#include "chrono_vehicle/driver/ChInteractiveDriverVSG.h"

namespace chrono {
namespace vehicle {

ChInteractiveDriverVSG::ChInteractiveDriverVSG(ChVehicleVisualSystemVSG& vsys) : ChInteractiveDriver(*vsys.m_vehicle) {
    vsys.m_driver = this;
}

ChInteractiveDriverVSG::~ChInteractiveDriverVSG() {}

void ChInteractiveDriverVSG::Initialize() {}

void ChInteractiveDriverVSG::IncreaseThrottle() {
    m_throttle_target = ChClamp(m_throttle_target + m_throttle_delta, 0.0, +1.0);
    if (m_throttle_target > 0)
        m_braking_target = ChClamp(m_braking_target - m_braking_delta * 3, 0.0, +1.0);
}

void ChInteractiveDriverVSG::DecreaseThrottle() {
    m_throttle_target = ChClamp(m_throttle_target - m_throttle_delta * 3, 0.0, +1.0);
    if (m_throttle_target <= 0)
        m_braking_target = ChClamp(m_braking_target + m_braking_delta, 0.0, +1.0);
}

void ChInteractiveDriverVSG::SteeringLeft() {
    m_steering_target = ChClamp(m_steering_target + m_steering_delta, -1.0, +1.0);
}

void ChInteractiveDriverVSG::SteeringRight() {
    m_steering_target = ChClamp(m_steering_target - m_steering_delta, -1.0, +1.0);
}

void ChInteractiveDriverVSG::SteeringCenter() {
    m_steering_target = 0.0;
}

void ChInteractiveDriverVSG::ReleasePedals() {
    m_throttle_target = 0.0;
    m_braking_target = 0.0;
    m_clutch_target = 0.0;
}

}  // namespace vehicle
}  // namespace chrono
