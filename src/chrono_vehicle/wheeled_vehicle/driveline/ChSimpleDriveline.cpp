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
// Simple driveline model. This template can be used to model a FWD, a RWD, or a
// 4WD driveline. It uses a constant front/rear torque split (a value between 0
// and 1 for 4WD, 0 for RWD, and 1 for FWD) and a simple model for a Torsen
// limited-slip differential (front and/or rear).
//
// =============================================================================

#include <cmath>

#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDriveline.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a default 4WD simple driveline.
// -----------------------------------------------------------------------------
ChSimpleDriveline::ChSimpleDriveline(const std::string& name)
    : ChDrivelineWV(name), m_connected(true), m_driveshaft_speed(0) {}

// -----------------------------------------------------------------------------
// Initialize the driveline subsystem.
// This function connects this driveline to the specified axles.
// -----------------------------------------------------------------------------
void ChSimpleDriveline::Initialize(std::shared_ptr<ChChassis> chassis,
                                   const ChAxleList& axles,
                                   const std::vector<int>& driven_axles) {
    assert(driven_axles.size() == 2);

    // Create the driveshaft
    ChDriveline::Initialize(chassis);

    m_driven_axles = driven_axles;

    // Grab handles to the suspension wheel shafts.
    m_front_left = axles[m_driven_axles[0]]->m_suspension->GetAxle(LEFT);
    m_front_right = axles[m_driven_axles[0]]->m_suspension->GetAxle(RIGHT);

    m_rear_left = axles[m_driven_axles[1]]->m_suspension->GetAxle(LEFT);
    m_rear_right = axles[m_driven_axles[1]]->m_suspension->GetAxle(RIGHT);
}

// -----------------------------------------------------------------------------
// This utility function implements a simple model of Torsen limited-slip
// differential with a max_bias:1 torque bias ratio.
// We hardcode the speed difference range over which the torque bias grows from
// a value of 1 to a value of max_bias to the interval [0.25, 0.5].
// -----------------------------------------------------------------------------
void differentialSplit(double torque,
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
// -----------------------------------------------------------------------------
void ChSimpleDriveline::Synchronize(double time, const DriverInputs& driver_inputs, double driveshaft_torque) {
    if (!m_connected)
        return;

    // Set driveshaft speed (output to transmission)
    double speed_front = 0.5 * (m_front_left->GetPos_dt() + m_front_right->GetPos_dt());
    double speed_rear = 0.5 * (m_rear_left->GetPos_dt() + m_rear_right->GetPos_dt());
    double alpha = GetFrontTorqueFraction();
    m_driveshaft_speed = alpha * speed_front + (1 - alpha) * speed_rear;

    // Split the input torque front/back.
    double torque_front = driveshaft_torque * GetFrontTorqueFraction();
    double torque_rear = driveshaft_torque - torque_front;

    // Split the axle torques for the corresponding left/right wheels and apply
    // them to the suspension wheel shafts.
    double torque_left;
    double torque_right;

    differentialSplit(torque_front, GetFrontDifferentialMaxBias(), m_front_left->GetPos_dt(),
                      m_front_right->GetPos_dt(), torque_left, torque_right);
    m_front_left->SetAppliedTorque(-torque_left);
    m_front_right->SetAppliedTorque(-torque_right);

    differentialSplit(torque_rear, GetRearDifferentialMaxBias(), m_rear_left->GetPos_dt(), m_rear_right->GetPos_dt(),
                      torque_left, torque_right);
    m_rear_left->SetAppliedTorque(-torque_left);
    m_rear_right->SetAppliedTorque(-torque_right);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChSimpleDriveline::GetSpindleTorque(int axle, VehicleSide side) const {
    if (!m_connected)
        return 0;

    if (axle == m_driven_axles[0]) {
        switch (side) {
            case LEFT:
                return -m_front_left->GetAppliedTorque();
            case RIGHT:
                return -m_front_right->GetAppliedTorque();
        }
    } else if (axle == m_driven_axles[1]) {
        switch (side) {
            case LEFT:
                return -m_rear_left->GetAppliedTorque();
            case RIGHT:
                return -m_rear_right->GetAppliedTorque();
        }
    }

    return 0;
}

// -----------------------------------------------------------------------------
void ChSimpleDriveline::Disconnect() {
    m_connected = false;
}

}  // end namespace vehicle
}  // end namespace chrono
