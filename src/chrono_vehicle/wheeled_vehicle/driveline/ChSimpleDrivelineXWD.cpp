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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Simple driveline model. This template can be used to model a XWD driveline.
// Number of axles can be 1 to X.
// It uses a constant torque split depending on the number of axles driven and a
// simple model for Torsen limited-slip differentials.
//
// =============================================================================

#include <cmath>

#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDrivelineXWD.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a default 4WD simple driveline.
// -----------------------------------------------------------------------------
ChSimpleDrivelineXWD::ChSimpleDrivelineXWD(const std::string& name) : ChDrivelineWV(name), m_connected(true) {}

// -----------------------------------------------------------------------------
// Initialize the driveline subsystem.
// This function connects this driveline to the specified axles.
// -----------------------------------------------------------------------------
void ChSimpleDrivelineXWD::Initialize(std::shared_ptr<ChChassis> chassis,
                                      const ChAxleList& axles,
                                      const std::vector<int>& driven_axles) {
    assert(axles.size() >= 1);

    // Create the driveshaft
    m_driveshaft = chrono_types::make_shared<ChShaft>();
    m_driveshaft->SetInertia(0.5);
    chassis->GetSystem()->AddShaft(m_driveshaft);

    m_driven_axles = driven_axles;

    for (int i = 0; i < driven_axles.size(); i++) {
        // Grab handles to the suspension wheel shafts.
        m_shaft_left.push_back(axles[m_driven_axles[i]]->m_suspension->GetAxle(LEFT));
        m_shaft_right.push_back(axles[m_driven_axles[i]]->m_suspension->GetAxle(RIGHT));
    }
}

// -----------------------------------------------------------------------------
// This utility function implements a simple model of Torsen limited-slip
// differential with a max_bias:1 torque bias ratio.
// We hardcode the speed difference range over which the torque bias grows from
// a value of 1 to a value of max_bias to the interval [0.25, 0.5].
// -----------------------------------------------------------------------------
void differentialSplitXWD(double torque,
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
void ChSimpleDrivelineXWD::Synchronize(double time, const DriverInputs& driver_inputs, double torque) {
    if (!m_connected)
        return;

    double alpha = 1.0 / double(m_shaft_left.size());

    // Enforce driveshaft speed 
    double driveshaft_speed = 0;
    for (int i = 0; i < m_shaft_left.size(); i++) {
        driveshaft_speed += alpha * 0.5 * (m_shaft_left[i]->GetPos_dt() + m_shaft_right[i]->GetPos_dt());
    }
    m_driveshaft->SetPos_dt(driveshaft_speed);

    // Split the input torque over all driven axles. 
    double torque_axle = alpha * torque;

    // Split the axle torques for the corresponding left/right wheels and apply
    // them to the suspension wheel shafts.

    for (int axle = 0; axle < m_shaft_left.size(); axle++) {
        double torque_left;
        double torque_right;
        differentialSplitXWD(torque_axle, GetDifferentialMaxBias(), m_shaft_left[axle]->GetPos_dt(),
                             m_shaft_right[axle]->GetPos_dt(), torque_left, torque_right);
        m_shaft_left[axle]->SetAppliedTorque(-torque_left);
        m_shaft_right[axle]->SetAppliedTorque(-torque_right);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChSimpleDrivelineXWD::GetSpindleTorque(int axle, VehicleSide side) const {
    if (!m_connected)
        return 0;

    switch (side) {
        case LEFT:
            return -m_shaft_left[axle]->GetAppliedTorque();
        case RIGHT:
            return -m_shaft_right[axle]->GetAppliedTorque();
    }

    return 0;
}

// -----------------------------------------------------------------------------
void ChSimpleDrivelineXWD::Disconnect() {
    m_connected = false;
}

}  // end namespace vehicle
}  // end namespace chrono
