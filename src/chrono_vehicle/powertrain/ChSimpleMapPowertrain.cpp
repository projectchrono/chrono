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
// Authors: Radu Serban, Mike Taylor, Asher Elmquist, Luning Fang
// =============================================================================
//
// Simple powertrain model template.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#include "chrono/core/ChMathematics.h"

#include "chrono_vehicle/powertrain/ChSimpleMapPowertrain.h"

namespace chrono {
namespace vehicle {

ChSimpleMapPowertrain::ChSimpleMapPowertrain(const std::string& name)
    : ChPowertrain(name), m_motor_speed(0), m_motor_torque(0), m_shaft_torque(0), m_motor_resistance_c0(0), m_motor_resistance_c1(0) {}

double ChSimpleMapPowertrain::GetMotorResistance() const {
    return m_motor_resistance_c0 + m_motor_resistance_c1 * m_motor_speed;
}

void ChSimpleMapPowertrain::Initialize(std::shared_ptr<ChChassis> chassis) {
    ChPowertrain::Initialize(chassis);

    // Let the derived class specify motor resistance coefficients
    SetMotorResistanceCoefficients(m_motor_resistance_c0, m_motor_resistance_c1);

    // Let the derived class specify the shift bands
    SetShiftPoints(m_shift_points);
    assert(m_shift_points.size() == m_gear_ratios.size() - 1);

    // Let the derived class set the engine maps
    SetEngineTorqueMaps(m_zero_throttle_map, m_full_throttle_map);
    assert(m_zero_throttle_map.GetPoints().size() > 0);
    assert(m_full_throttle_map.GetPoints().size() > 0);
}

void ChSimpleMapPowertrain::Synchronize(double time, const DriverInputs& driver_inputs, double shaft_speed) {
    // Automatic gear selection (based on ideal shift points)
    if (m_transmission_mode == TransmissionMode::AUTOMATIC && m_drive_mode == DriveMode::FORWARD) {
        if (m_motor_speed > m_shift_points[m_current_gear].second) {
            // upshift if possible
            if (m_current_gear < m_gear_ratios.size() - 1) {
                SetGear(m_current_gear + 1);
            }
        } else if (m_motor_speed < m_shift_points[m_current_gear].first) {
            // downshift if possible
            if (m_current_gear > 1) {
                SetGear(m_current_gear - 1);
            }
        }
    }

    // Calculate engine speed and clamp to specified maximum
    m_motor_speed = std::abs(shaft_speed) / m_current_gear_ratio;
    ChClampValue(m_motor_speed, 0.0, GetMaxEngineSpeed());

    // Motor torque is linearly interpolated by throttle value
    double throttle = driver_inputs.m_throttle;
    double fullThrottleTorque = m_full_throttle_map.Get_y(m_motor_speed);
    double zeroThrottleTorque = m_zero_throttle_map.Get_y(m_motor_speed);
    m_motor_torque = zeroThrottleTorque * (1 - throttle) + fullThrottleTorque * (throttle);

    // Apply motor reistance
    m_motor_torque -= m_motor_resistance_c0 + m_motor_resistance_c1 * m_motor_speed;

    // The torque at motor shaft
    m_shaft_torque = m_motor_torque / m_current_gear_ratio;
}

}  // end namespace vehicle
}  // end namespace chrono
