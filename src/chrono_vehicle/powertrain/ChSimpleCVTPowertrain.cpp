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

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSimpleCVTPowertrain::ChSimpleCVTPowertrain(const std::string& name)
    : ChPowertrain(name), m_motorSpeed(0), m_motorTorque(0), m_shaftTorque(0) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSimpleCVTPowertrain::Initialize(std::shared_ptr<ChBody> chassis, std::shared_ptr<ChShaft> driveshaft) {
    m_current_gear_ratio = GetForwardGearRatio();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSimpleCVTPowertrain::SetDriveMode(ChPowertrain::DriveMode mode) {
    m_drive_mode = mode;
    switch (mode) {
        case FORWARD:
            m_current_gear_ratio = GetForwardGearRatio();
            break;
        case REVERSE:
            m_current_gear_ratio = GetReverseGearRatio();
            break;
        case NEUTRAL:
            m_current_gear_ratio = 1e20;
            break;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSimpleCVTPowertrain::Synchronize(double time, double throttle, double shaft_speed) {
    // The motor speed is the shaft speed multiplied by gear ratio inversed:
    m_motorSpeed = shaft_speed / m_current_gear_ratio;

    // The torque depends on a hyperbolic speed-torque curve of the motor
    // like in DC motors or combustion engines with CVT gearbox.
    if (m_motorSpeed <= GetCriticalSpeed()) {
        m_motorTorque = GetMaxTorque();
    } else {
        m_motorTorque = GetMaxPower() / (CH_C_2PI * m_motorSpeed);
    }

    // Motor torque is linearly modulated by throttle gas value:
    m_motorTorque *= throttle;

    // The torque at motor shaft:
    m_shaftTorque = m_motorTorque / m_current_gear_ratio;
}

}  // end namespace vehicle
}  // end namespace chrono
