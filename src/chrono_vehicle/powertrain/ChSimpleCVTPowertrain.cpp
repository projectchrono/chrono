// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Simple powertrain model template.
// - hyperbolical speed-torque curve like a CVT gearbox
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "chrono_vehicle/powertrain/ChSimpleCVTPowertrain.h"

namespace chrono {
namespace vehicle {

ChSimpleCVTPowertrain::ChSimpleCVTPowertrain(const std::string& name)
    : ChPowertrain(name), m_motorSpeed(0), m_motorTorque(0), m_shaftTorque(0), m_critical_speed(1e4) {}

void ChSimpleCVTPowertrain::Initialize(std::shared_ptr<ChChassis> chassis) {
    ChPowertrain::Initialize(chassis);
    m_critical_speed = GetMaxPower() / GetMaxTorque();
}

void ChSimpleCVTPowertrain::Synchronize(double time, const DriverInputs& driver_inputs, double shaft_speed) {
    // The motor speed is the shaft speed multiplied by gear ratio inversed
    m_motorSpeed = std::abs(shaft_speed / m_current_gear_ratio);

    // The torque depends on a hyperbolic speed-torque curve of the motor
    // like in DC motors or combustion engines with CVT gearbox.
    if (m_motorSpeed <= m_critical_speed) {
        m_motorTorque = GetMaxTorque();
    } else {
        m_motorTorque = GetMaxPower() / m_motorSpeed;
    }

    // Limit the speed range
    if (m_motorSpeed >= GetMaxSpeed()) {
        m_motorTorque = 0.0;
    }

    // Motor torque is linearly modulated by throttle gas value:
    m_motorTorque *= driver_inputs.m_throttle;

    // The torque at motor shaft
    m_shaftTorque = m_motorTorque / m_current_gear_ratio;
}

}  // end namespace vehicle
}  // end namespace chrono
