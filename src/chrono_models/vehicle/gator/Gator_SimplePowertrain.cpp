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
// Authors: Radu Serban
// =============================================================================
//
// Simple powertrain model for the Gator vehicle.
// - linear speed-torque curve
// - no torque converter
// - no transmission box (single forward gear, single reverse gear)
//
// =============================================================================

#include "chrono_models/vehicle/gator/Gator_SimplePowertrain.h"

namespace chrono {
namespace vehicle {
namespace gator {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Gator_SimplePowertrain::m_max_torque = 200;                      // N.m
const double Gator_SimplePowertrain::m_max_speed = 3500 * (CH_C_2PI / 60.0);  // rad/s
const double Gator_SimplePowertrain::m_fwd_gear_ratio = 0.07;
const double Gator_SimplePowertrain::m_rev_gear_ratio = -0.04;

// -----------------------------------------------------------------------------
Gator_SimplePowertrain::Gator_SimplePowertrain(const std::string& name)
    : ChPowertrain(name), m_motorSpeed(0), m_motorTorque(0), m_shaftTorque(0) {}

void Gator_SimplePowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear_ratio;
    fwd.push_back(m_fwd_gear_ratio);
}

void Gator_SimplePowertrain::Synchronize(double time, const DriverInputs& driver_inputs, double shaft_speed) {
    // The motor speed is the shaft speed multiplied by gear ratio inversed
    m_motorSpeed = std::abs(shaft_speed) / std::abs(m_current_gear_ratio);

    // DC motor model (throttle modulates output torque)
    m_motorTorque = m_max_torque - m_motorSpeed * (m_max_torque / m_max_speed);
    m_motorTorque *= driver_inputs.m_throttle;

    // The torque at motor shaft
    m_shaftTorque = m_motorTorque / m_current_gear_ratio;

    ////std::cout << throttle                                            //
    ////          << "      " << m_motorSpeed << "  " << shaft_speed     //
    ////          << "      " << m_motorTorque << "  " << m_shaftTorque  //
    ////          << std::endl;
}

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono
