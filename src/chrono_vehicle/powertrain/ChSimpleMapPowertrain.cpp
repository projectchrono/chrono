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
// Authors: Radu Serban, Mike Taylor, Asher Elmquist
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

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSimpleMapPowertrain::ChSimpleMapPowertrain()
    : ChPowertrain(), m_initialized(false), m_automatic(true), m_motor_speed(0), m_motor_torque(0), m_shaft_torque(0) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSimpleMapPowertrain::Initialize(std::shared_ptr<ChBody> chassis, std::shared_ptr<ChShaft> driveshaft) {
    // Let the derived class specify the gear ratios
    SetGearRatios(m_gear_ratios, m_rev_gear_ratio);
    assert(m_gear_ratios.size() > 0);

    // Let the derived class specify the shift bands
    SetShiftPoints(m_shift_points);
    assert(m_shift_points.size() == m_gear_ratios.size());

    // Let the derived class set the engine maps
    SetEngineTorqueMaps(m_zero_throttle_map, m_full_throttle_map);
    assert(m_zero_throttle_map.GetPoints().size() > 0);
    assert(m_full_throttle_map.GetPoints().size() > 0);

    // Initialize to 1st gear
    m_current_gear = 0;
    m_current_gear_ratio = m_gear_ratios[0];

    m_initialized = true;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSimpleMapPowertrain::EnableAutomaticTransmission(bool automatic) {
    m_automatic = automatic;

    if (m_initialized) {
        // Reset to 1st gear
        m_current_gear = 0;
        m_current_gear_ratio = m_gear_ratios[0];
    }
}

void ChSimpleMapPowertrain::SetDriveMode(ChPowertrain::DriveMode mode) {
    if (!m_initialized || m_drive_mode == mode)
        return;

    m_drive_mode = mode;

    switch (mode) {
        case DriveMode::FORWARD:
            m_current_gear = 0;
            m_current_gear_ratio = m_gear_ratios[0];
            break;
        case DriveMode::REVERSE:
            m_current_gear_ratio = m_rev_gear_ratio;
            break;
        case DriveMode::NEUTRAL:
            m_current_gear_ratio = 1e20;
            break;
    }
}

int ChSimpleMapPowertrain::GetCurrentTransmissionGear() const {
    if (m_drive_mode == DriveMode::REVERSE)
        return 0;

    return m_current_gear + 1;
}

// Set the current forward gear (1, 2, ...).  A zero latency is assumed.
void ChSimpleMapPowertrain::SetForwardGear(int igear) {
    if (!m_initialized || m_automatic || m_drive_mode != DriveMode::FORWARD)
        return;

    assert(igear >= 1);
    assert(igear <= m_gear_ratios.size());

    m_current_gear = igear - 1;
    m_current_gear_ratio = m_gear_ratios[m_current_gear];
}

// -----------------------------------------------------------------------------
// Simple automatic gear selection, based on ideal shift points
// -----------------------------------------------------------------------------
void ChSimpleMapPowertrain::CheckShift() {
    if (m_motor_speed > m_shift_points[m_current_gear].second) {
        // Try to up-shift
        if (m_current_gear + 1 < m_gear_ratios.size()) {
            m_current_gear++;
            m_current_gear_ratio = m_gear_ratios[m_current_gear];
        }
    } else if (m_motor_speed < m_shift_points[m_current_gear].first) {
        // Try to down-shift
        if (m_current_gear > 0) {
            m_current_gear--;
            m_current_gear_ratio = m_gear_ratios[m_current_gear];
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSimpleMapPowertrain::Synchronize(double time, double throttle, double shaft_speed) {
    if (m_automatic && m_drive_mode == DriveMode::FORWARD)
        CheckShift();

    // Calculate engine speed and clamp to specified maximum:
    m_motor_speed = shaft_speed / m_current_gear_ratio;
    ChClampValue(m_motor_speed, 0.0, GetMaxEngineSpeed());

    // Motor torque is linearly interpolated by throttle value:
    double fullThrottleTorque = m_full_throttle_map.Get_y(m_motor_speed);
    double zeroThrottleTorque = m_zero_throttle_map.Get_y(m_motor_speed);
    m_motor_torque = zeroThrottleTorque * (1 - throttle) + fullThrottleTorque * (throttle);

    // The torque at motor shaft:
    m_shaft_torque = m_motor_torque / m_current_gear_ratio;
}

}  // end namespace vehicle
}  // end namespace chrono
