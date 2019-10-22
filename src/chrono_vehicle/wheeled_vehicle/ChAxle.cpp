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

ChAxle::ChAxle() : m_steering_index(-1) {}

void ChAxle::SetOutput(bool state) {
    m_suspension->SetOutput(state);
    m_brake_left->SetOutput(state);
    m_brake_right->SetOutput(state);
    for (auto& wheel : m_wheels) {
        wheel->SetOutput(state);
    }
    if (m_antirollbar) {
        m_antirollbar->SetOutput(state);
    }
}

void ChAxle::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                        const ChVector<>& susp_location,
                        const ChVector<>& arb_location,
                        std::shared_ptr<ChBody> tierod_body,
                        int steering_index,
                        double wheel_separation,
                        double left_ang_vel,
                        double right_ang_vel) {
    m_suspension->Initialize(chassis, susp_location, tierod_body, steering_index, left_ang_vel, right_ang_vel);
    m_brake_left->Initialize(m_suspension, LEFT);
    m_brake_right->Initialize(m_suspension, RIGHT);
    if (wheel_separation > 0) {
        assert(m_wheels.size() == 4);
        m_wheels[0]->Initialize(m_suspension->GetSpindle(LEFT), LEFT, -wheel_separation / 2);    // inner left
        m_wheels[1]->Initialize(m_suspension->GetSpindle(RIGHT), RIGHT, -wheel_separation / 2);  // inner right
        m_wheels[2]->Initialize(m_suspension->GetSpindle(LEFT), LEFT, +wheel_separation / 2);    // outer left
        m_wheels[3]->Initialize(m_suspension->GetSpindle(RIGHT), RIGHT, +wheel_separation / 2);  // outer right
    } else {
        assert(m_wheels.size() == 2);
        m_wheels[0]->Initialize(m_suspension->GetSpindle(LEFT), LEFT);    // left
        m_wheels[1]->Initialize(m_suspension->GetSpindle(RIGHT), RIGHT);  // right
    }
    if (m_antirollbar) {
        assert(m_suspension->IsIndependent());
        m_antirollbar->Initialize(chassis, arb_location, m_suspension->GetLeftBody(), m_suspension->GetRightBody());
    }
}

void ChAxle::Synchronize(double braking) {
    // Synchronize suspension subsystem (prepare to accept tire forces)
    m_suspension->Synchronize();

    // Let the wheel subsystems get tire forces and pass them to their associated suspension.
    for (auto& wheel : m_wheels) {
        wheel->Synchronize();
    }

    // Apply braking input.
    m_brake_left->Synchronize(braking);
    m_brake_right->Synchronize(braking);
}

std::shared_ptr<ChWheel> ChAxle::GetWheel(VehicleSide side, WheelLocation location) const {
    assert((location == SINGLE && m_wheels.size() == 2) || (location != SINGLE && m_wheels.size() == 4));
    if (location == SINGLE)
        return m_wheels[side];
    return m_wheels[2 * (location - 1) + side];
}

}  // end namespace vehicle
}  // end namespace chrono
