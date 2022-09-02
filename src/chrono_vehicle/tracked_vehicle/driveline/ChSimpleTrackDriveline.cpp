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
// Simple driveline model for a tracked vehicle. This template splits the input
// torque to the left and right tracks using a simple analytical model of a
// Torsen limited-slip differential and the given driver steering input.
//
// =============================================================================

#include <cmath>

#include "chrono_vehicle/tracked_vehicle/driveline/ChSimpleTrackDriveline.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a default simple track driveline.
// -----------------------------------------------------------------------------
ChSimpleTrackDriveline::ChSimpleTrackDriveline(const std::string& name) : ChDrivelineTV(name), m_connected(true) {}

// -----------------------------------------------------------------------------
// Initialize the driveline subsystem.
// This function connects this driveline subsystem to the sprockets of the
// two track assembly subsystems.
// -----------------------------------------------------------------------------
void ChSimpleTrackDriveline::Initialize(std::shared_ptr<ChChassis> chassis,
                                        std::shared_ptr<ChTrackAssembly> track_left,
                                        std::shared_ptr<ChTrackAssembly> track_right) {
    // Create the driveshaft
    m_driveshaft = chrono_types::make_shared<ChShaft>();
    m_driveshaft->SetInertia(0.5);
    chassis->GetSystem()->AddShaft(m_driveshaft);

    // Grab handles to the sprocket shafts.
    m_shaft_left = track_left->GetSprocket()->GetAxle();
    m_shaft_right = track_right->GetSprocket()->GetAxle();
}

// -----------------------------------------------------------------------------
// This utility function implements a simple model of Torsen limited-slip
// differential with a max_bias:1 torque bias ratio.
// We hardcode the speed difference range over which the torque bias grows from
// a value of 1 to a value of max_bias to the interval [0.25, 0.5].
// -----------------------------------------------------------------------------
static void differentialSplit(double torque,
                              double max_bias,
                              double speed_left,
                              double speed_right,
                              double& torque_left,
                              double& torque_right) {
    double diff = std::abs(speed_left - speed_right);

    // The bias grows from 1 at diff=0.25 to max_bias at diff=0.5
    double bias = 1;
    if (diff > 0.5)
        bias = max_bias;
    else if (diff > 0.25)
        bias = 4 * (max_bias - 1) * diff + (2 - max_bias);

    // Split torque to the slow and fast wheels.
    double alpha = bias / (1 + bias);
    double slow = alpha * torque;
    double fast = torque - slow;

    if (std::abs(speed_left) < std::abs(speed_right)) {
        torque_left = slow;
        torque_right = fast;
    } else {
        torque_left = fast;
        torque_right = slow;
    }
}

// -----------------------------------------------------------------------------
void ChSimpleTrackDriveline::Synchronize(double time, const DriverInputs& driver_inputs, double torque) {
    if (!m_connected)
        return;

    // Enforce driveshaft speed 
    double driveshaft_speed = 0.5 * (m_shaft_left->GetPos_dt() + m_shaft_right->GetPos_dt());
    m_driveshaft->SetPos_dt(driveshaft_speed);

    // Split the axle torques for the corresponding left/right sprockets and apply
    // them to the sprocket axle shafts.
    double torque_left;
    double torque_right;

    differentialSplit(torque, GetDifferentialMaxBias(), m_shaft_left->GetPos_dt(), m_shaft_right->GetPos_dt(),
                      torque_left, torque_right);

    // Include steering.
    double steering = driver_inputs.m_steering;
    double factor_left = 1;
    double factor_right = 1;

    if (steering > 0)
        factor_left = m_gyration_mode ? (1 - 2 * steering) : (1 - steering);
    if (steering < 0)
        factor_right = m_gyration_mode ? (1 + 2 * steering) : (1 + steering);

    // Apply torques to the sprocket shafts.
    m_shaft_left->SetAppliedTorque(-torque_left * factor_left);
    m_shaft_right->SetAppliedTorque(-torque_right * factor_right);
}

// -----------------------------------------------------------------------------
double ChSimpleTrackDriveline::GetSprocketTorque(VehicleSide side) const {
    if (!m_connected)
        return 0;

    switch (side) {
        case LEFT:
            return -m_shaft_left->GetAppliedTorque();
        case RIGHT:
            return -m_shaft_right->GetAppliedTorque();
    }

    return 0;
}

// -----------------------------------------------------------------------------
double ChSimpleTrackDriveline::GetSprocketSpeed(VehicleSide side) const {
    switch (side) {
        case LEFT:
            return -m_shaft_left->GetPos_dt();
        case RIGHT:
            return -m_shaft_right->GetPos_dt();
    }

    return 0;
}

// -----------------------------------------------------------------------------
void ChSimpleTrackDriveline::Disconnect() {
    m_connected = false;
}

}  // end namespace vehicle
}  // end namespace chrono
