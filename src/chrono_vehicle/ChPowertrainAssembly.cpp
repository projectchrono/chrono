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
// Implementation of a vehicle powertrain assembly.
// This is an aggregate of an engine and a transmission.
//
// =============================================================================

#include "chrono_vehicle/ChPowertrainAssembly.h"

namespace chrono {
namespace vehicle {

void ChPowertrainAssembly::Initialize(std::shared_ptr<ChChassis> chassis) {
    m_engine->Initialize(chassis);
    m_transmission->Initialize(chassis);
}

void ChPowertrainAssembly::Synchronize(double time, const DriverInputs& driver_inputs, double driveshaft_speed) {
    double motorshaft_torque = m_engine->GetOutputMotorshaftTorque();
    double motorshaft_speed = m_transmission->GetOutputMotorshaftSpeed();

    m_engine->Synchronize(time, driver_inputs, motorshaft_speed);
    m_transmission->Synchronize(time, driver_inputs, motorshaft_torque, driveshaft_speed);
}

void ChPowertrainAssembly::Advance(double step) {
    m_engine->Advance(step);
    m_transmission->Advance(step);
}

}  // end namespace vehicle
}  // end namespace chrono
