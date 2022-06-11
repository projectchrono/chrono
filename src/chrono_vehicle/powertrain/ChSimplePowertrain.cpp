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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Simple powertrain model template.
// - trivial speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "chrono_vehicle/powertrain/ChSimplePowertrain.h"

namespace chrono {
namespace vehicle {

ChSimplePowertrain::ChSimplePowertrain(const std::string& name)
    : ChPowertrain(name), m_motorSpeed(0), m_motorTorque(0), m_shaftTorque(0) {}

void ChSimplePowertrain::Synchronize(double time, const DriverInputs& driver_inputs, double shaft_speed) {
    // The motor speed is the shaft speed multiplied by gear ratio inversed:
    m_motorSpeed = shaft_speed / m_current_gear_ratio;

    // The torque depends on speed-torque curve of the motor; here we assume a
    // very simplified model a bit like in DC motors.
    m_motorTorque = GetMaxTorque() - m_motorSpeed * (GetMaxTorque() / GetMaxSpeed());

    // Motor torque is linearly modulated by throttle gas value:
    m_motorTorque *= driver_inputs.m_throttle;

    // The torque at motor shaft:
    m_shaftTorque = m_motorTorque / m_current_gear_ratio;
}

}  // end namespace vehicle
}  // end namespace chrono
