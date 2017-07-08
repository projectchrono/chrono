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
// Authors: Radu Serban, Mike Taylor
// =============================================================================
//
// Simple powertrain model for the M113 vehicle.
// - both power and torque limited
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "chrono_models/vehicle/m113a/M113a_SimplePowertrain.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113a_SimplePowertrain::M113a_SimplePowertrain() : ChPowertrain(), m_motorSpeed(0), m_motorTorque(0), m_shaftTorque(0) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113a_SimplePowertrain::Initialize(std::shared_ptr<ChBody> chassis, std::shared_ptr<ChShaft> driveshaft) {
    m_current_gear_ratio = 1;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113a_SimplePowertrain::SetDriveMode(ChPowertrain::DriveMode mode) {
    m_drive_mode = mode;
    switch (mode) {
    case FORWARD:
        m_current_gear_ratio = 1;
        break;
    case REVERSE:
        m_current_gear_ratio = -1;
        break;
    case NEUTRAL:
        m_current_gear_ratio = 1e20;
        break;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113a_SimplePowertrain::Synchronize(double time, double throttle, double shaft_speed) {
    // The motor speed is the shaft speed multiplied by gear ratio inversed:
    m_motorSpeed = std::abs(shaft_speed / m_current_gear_ratio);

    // Assume the motor is both torque and power limited.
    // Max torque = 2x7021 ft-lbs (19038.4 Nm)
    // Max power = 200 hp
    // Crossing point = 7.8335 rad/s
    m_motorTorque = m_motorSpeed <= 7.8335 ? 19038.4 : (149136.762 / m_motorSpeed);

    // Motor torque is linearly modulated by throttle gas value:
    m_motorTorque *= throttle;

    // The torque at motor shaft:
    m_shaftTorque = m_motorTorque / m_current_gear_ratio;
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
