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
// Gator simple driveline model.
//
// =============================================================================

#include "chrono_models/vehicle/gator/Gator_SimpleDriveline.h"

namespace chrono {
namespace vehicle {
namespace gator {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Gator_SimpleDriveline::m_diff_bias = 2.0;

// -----------------------------------------------------------------------------
Gator_SimpleDriveline::Gator_SimpleDriveline(const std::string& name)
    : ChDrivelineWV(name), m_connected(true), m_driveshaft_speed(0) {}

void Gator_SimpleDriveline::Initialize(std::shared_ptr<ChChassis> chassis,
                                       const ChAxleList& axles,
                                       const std::vector<int>& driven_axles) {
    assert(axles.size() == 2);
    assert(driven_axles.size() == 1);

    m_driven_axles = driven_axles;

    // Grab handles to the suspension wheel shafts.
    m_left = axles[m_driven_axles[0]]->m_suspension->GetAxle(LEFT);
    m_right = axles[m_driven_axles[0]]->m_suspension->GetAxle(RIGHT);
}

// -----------------------------------------------------------------------------
void Gator_SimpleDriveline::Synchronize(double time, const DriverInputs& driver_inputs, double torque) {
    if (!m_connected)
        return;

    // Enforce driveshaft speed
    m_driveshaft_speed = 0.5 * (m_left->GetPos_dt() + m_right->GetPos_dt());

    // Split the driveshaft torque for the corresponding left/right wheels.
    // Use a siple model of a Torsten limited-slip differential with a max_bias:1 torque bias ratio.
    double speed_left = m_left->GetPos_dt();
    double speed_right = m_right->GetPos_dt();
    double diff = std::abs(speed_left - speed_right);

    // The bias grows from 1 at diff=0.25 to m_diff_bias at diff=0.5
    double bias = 1;
    if (diff > 0.5)
        bias = m_diff_bias;
    else if (diff > 0.25)
        bias = 4 * (m_diff_bias - 1) * diff + (2 - m_diff_bias);

    // Split torque to the slow and fast wheels.
    double alpha = bias / (1 + bias);
    double slow = alpha * torque;
    double fast = torque - slow;

    double torque_left;
    double torque_right;
    if (std::abs(speed_left) < std::abs(speed_right)) {
        torque_left = slow;
        torque_right = fast;
    } else {
        torque_left = fast;
        torque_right = slow;
    }

    // Apply torques to left and right shafts
    m_left->SetAppliedTorque(-torque_left);
    m_right->SetAppliedTorque(-torque_right);
}

// -----------------------------------------------------------------------------
double Gator_SimpleDriveline::GetSpindleTorque(int axle, VehicleSide side) const {
    if (!m_connected)
        return 0;

    if (axle != m_driven_axles[0])
        return 0;

    if (side == VehicleSide::LEFT)
        return -m_left->GetAppliedTorque();

    return -m_right->GetAppliedTorque();
}

// -----------------------------------------------------------------------------
void Gator_SimpleDriveline::Disconnect() {
    m_connected = false;
}

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono
