// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Marcel Offermans
// =============================================================================
//
// Automatic transmission model template based on a simple gear-shifting model.
//
// =============================================================================

#include "chrono/core/ChMathematics.h"

#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {

ChAutomaticTransmissionSimpleMap::ChAutomaticTransmissionSimpleMap(const std::string& name)
    : ChTransmission(name), m_motorshaft_speed(0), m_driveshaft_torque(0) {}

void ChAutomaticTransmissionSimpleMap::Initialize(std::shared_ptr<ChChassis> chassis) {
    ChTransmission::Initialize(chassis);

    // If we have more than one forward gear...
    if (m_gear_ratios.size() > 2) {
        // ... we let the derived class specify the shift bands.
        SetShiftPoints(m_shift_points);
        // Make sure we have as many shift points as we have forward gears.
        assert(m_shift_points.size() == m_gear_ratios.size() - 1);
    }
}

void ChAutomaticTransmissionSimpleMap::Synchronize(double time,
                                                   const DriverInputs& driver_inputs,
                                                   double motorshaft_torque,
                                                   double driveshaft_speed) {
    // Automatic gear selection (based on ideal shift points) for current motorshaft speed
    if (m_mode == Mode::AUTOMATIC && m_drive_mode == DriveMode::FORWARD && m_current_gear > 0) {
        if ((m_current_gear < (m_gear_ratios.size() - 1)) &&
            (m_motorshaft_speed > m_shift_points[m_current_gear - 1].second)) {
            SetGear(m_current_gear + 1);
        } else if ((m_current_gear > 1) && (m_motorshaft_speed < m_shift_points[m_current_gear - 1].first)) {
            SetGear(m_current_gear - 1);
        }
    }

    // Set speed of the motorshaft (transmission output to the engine)
    m_motorshaft_speed = std::abs(driveshaft_speed) / m_current_gear_ratio;

    // Set torque at driveshaft (transmission output to the driveline)
    m_driveshaft_torque = motorshaft_torque / m_current_gear_ratio;
}

}  // end namespace vehicle
}  // end namespace chrono
