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
// Class representing a wheeled vehicle axle. A ChAxle encapsulates a suspension
// subsystem, (optionally) an anti-roll-bar subsystem, two brakes (left/right),
// and a varying number of wheels.
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/ChAxle.h"

namespace chrono {
namespace vehicle {

ChAxle::ChAxle() {}

void ChAxle::SetOutput(bool state) {
    m_suspension->SetOutput(state);
    if (m_brake_left && m_brake_right) {
        m_brake_left->SetOutput(state);
        m_brake_right->SetOutput(state);
    }
    for (auto& wheel : m_wheels) {
        wheel->SetOutput(state);
    }
    if (m_antirollbar) {
        m_antirollbar->SetOutput(state);
    }
}

void ChAxle::Initialize(std::shared_ptr<ChChassis> chassis,
                        std::shared_ptr<ChSubchassis> subchassis,
                        std::shared_ptr<ChSteering> steering,
                        const ChVector<>& susp_location,
                        const ChVector<>& arb_location,
                        double wheel_separation,
                        double left_ang_vel,
                        double right_ang_vel) {
    m_suspension->Initialize(chassis, subchassis, steering, susp_location, left_ang_vel, right_ang_vel);
    if (m_brake_left && m_brake_right) {
        m_brake_left->Initialize(chassis, m_suspension, LEFT);
        m_brake_right->Initialize(chassis, m_suspension, RIGHT);
    }
    if (wheel_separation > 0) {
        assert(m_wheels.size() == 4);
        m_wheels[0]->Initialize(chassis, m_suspension->GetSpindle(LEFT), LEFT, -wheel_separation / 2);    // inner left
        m_wheels[1]->Initialize(chassis, m_suspension->GetSpindle(RIGHT), RIGHT, -wheel_separation / 2);  // inner right
        m_wheels[2]->Initialize(chassis, m_suspension->GetSpindle(LEFT), LEFT, +wheel_separation / 2);    // outer left
        m_wheels[3]->Initialize(chassis, m_suspension->GetSpindle(RIGHT), RIGHT, +wheel_separation / 2);  // outer right
    } else {
        assert(m_wheels.size() == 2);
        m_wheels[0]->Initialize(chassis, m_suspension->GetSpindle(LEFT), LEFT);    // left
        m_wheels[1]->Initialize(chassis, m_suspension->GetSpindle(RIGHT), RIGHT);  // right
    }
    if (m_antirollbar) {
        assert(m_suspension->IsIndependent());
        m_antirollbar->Initialize(chassis, m_suspension, arb_location);
    }
}

void ChAxle::Synchronize(double time, const DriverInputs& driver_inputs) {
    // Synchronize suspension subsystem (prepare to accept tire forces)
    m_suspension->Synchronize();

    // Let the wheel subsystems get tire forces and pass them to their associated suspension.
    for (auto& wheel : m_wheels) {
        wheel->Synchronize();
    }

    // Apply braking input.
    if (m_brake_left && m_brake_right) {
        m_brake_left->Synchronize(driver_inputs.m_braking);
        m_brake_right->Synchronize(driver_inputs.m_braking);
    }
}

std::shared_ptr<ChWheel> ChAxle::GetWheel(VehicleSide side, WheelLocation location) const {
    assert((location == SINGLE && m_wheels.size() == 2) || (location != SINGLE && m_wheels.size() == 4));
    if (location == SINGLE)
        return m_wheels[side];
    return m_wheels[2 * (location - 1) + side];
}

std::shared_ptr<ChBrake> ChAxle::GetBrake(VehicleSide side) const {
    if (side == VehicleSide::LEFT)
        return m_brake_left;
    return m_brake_right;
}

}  // end namespace vehicle
}  // end namespace chrono
