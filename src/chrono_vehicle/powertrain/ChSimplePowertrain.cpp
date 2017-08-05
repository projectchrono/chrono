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

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSimplePowertrain::ChSimplePowertrain(const std::string& name) : ChPowertrain(name), m_motorSpeed(0), m_motorTorque(0), m_shaftTorque(0) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSimplePowertrain::Initialize(std::shared_ptr<ChBody> chassis, std::shared_ptr<ChShaft> driveshaft) {
    m_current_gear_ratio = GetForwardGearRatio();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSimplePowertrain::SetDriveMode(ChPowertrain::DriveMode mode) {
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
void ChSimplePowertrain::Synchronize(double time, double throttle, double shaft_speed) {
    // The motor speed is the shaft speed multiplied by gear ratio inversed:
    m_motorSpeed = shaft_speed / m_current_gear_ratio;

    // The torque depends on speed-torque curve of the motor; here we assume a
    // very simplified model a bit like in DC motors.
    m_motorTorque = GetMaxTorque() - m_motorSpeed * (GetMaxTorque() / GetMaxSpeed());

    // Motor torque is linearly modulated by throttle gas value:
    m_motorTorque *= throttle;

    // The torque at motor shaft:
    m_shaftTorque = m_motorTorque / m_current_gear_ratio;
}

}  // end namespace vehicle
}  // end namespace chrono
