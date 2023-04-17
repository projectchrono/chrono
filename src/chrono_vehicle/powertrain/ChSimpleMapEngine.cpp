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
// Authors: Radu Serban
// =============================================================================
//
// Simple engine model based on torque-speed engine maps
//
// =============================================================================

#include "chrono/core/ChMathematics.h"

#include "chrono_vehicle/powertrain/ChSimpleMapEngine.h"

namespace chrono {
namespace vehicle {

ChSimpleMapEngine::ChSimpleMapEngine(const std::string& name) : ChEngine(name), m_motor_speed(0), m_motor_torque(0) {}

void ChSimpleMapEngine::Initialize(std::shared_ptr<ChChassis> chassis) {
    ChEngine::Initialize(chassis);

    // Let the derived class set the engine maps
    SetEngineTorqueMaps(m_zero_throttle_map, m_full_throttle_map);
    assert(m_zero_throttle_map.GetPoints().size() > 0);
    assert(m_full_throttle_map.GetPoints().size() > 0);
}

void ChSimpleMapEngine::Synchronize(double time, const DriverInputs& driver_inputs, double shaft_speed) {
    // Clamp shaft speed to specified maximum
    m_motor_speed = ChClamp(shaft_speed, 0.0, GetMaxEngineSpeed());

    // Motor torque is linearly interpolated by throttle value
    double throttle = driver_inputs.m_throttle;
    double fullThrottleTorque = m_full_throttle_map.Get_y(m_motor_speed);
    double zeroThrottleTorque = m_zero_throttle_map.Get_y(m_motor_speed);
    m_motor_torque = zeroThrottleTorque * (1 - throttle) + fullThrottleTorque * (throttle);

    // Apply the motor torque on the motorshaft
    m_motorshaft->SetAppliedTorque(m_motor_torque);
}

}  // end namespace vehicle
}  // end namespace chrono
