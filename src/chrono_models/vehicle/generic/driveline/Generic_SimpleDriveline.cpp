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
// Authors: Radu Serban, Mike Taylor
// =============================================================================
//
// Simple driveline model for a single axle open differential.
//
// =============================================================================

#include <cmath>

#include "chrono_models/vehicle/generic/driveline/Generic_SimpleDriveline.h"

namespace chrono {
namespace vehicle {
namespace generic {
// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Generic_SimpleDriveline::m_conicalgear_ratio = -0.2433;

// -----------------------------------------------------------------------------
// Construct a 2WD open differential simple driveline.
// -----------------------------------------------------------------------------
Generic_SimpleDriveline::Generic_SimpleDriveline(const std::string& name)
    : ChDrivelineWV(name), m_connected(true), m_driveshaft_speed(0) {}

// -----------------------------------------------------------------------------
// Initialize the driveline subsystem.
// This function connects this driveline subsystem to the specified axles.
// -----------------------------------------------------------------------------
void Generic_SimpleDriveline::Initialize(std::shared_ptr<ChChassis> chassis,
                                         const ChAxleList& axles,
                                         const std::vector<int>& driven_axles) {
    assert(axles.size() >= 1);

    m_driven_axles = driven_axles;

    // Grab handles to the suspension wheel shafts.
    m_driven_left = axles[m_driven_axles[0]]->m_suspension->GetAxle(LEFT);
    m_driven_right = axles[m_driven_axles[0]]->m_suspension->GetAxle(RIGHT);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Generic_SimpleDriveline::Synchronize(double time, const DriverInputs& driver_inputs, double torque) {
    if (!m_connected)
        return;

    // Enforce driveshaft speed 
    m_driveshaft_speed = 0.5 * (m_driven_left->GetPos_dt() + m_driven_right->GetPos_dt());
    m_driveshaft_speed /= m_conicalgear_ratio;

    // Split the input torque front/back.
    double torque_drive = -torque / m_conicalgear_ratio;

    m_driven_left->SetAppliedTorque(-torque_drive / 2);
    m_driven_right->SetAppliedTorque(-torque_drive / 2);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double Generic_SimpleDriveline::GetSpindleTorque(int axle, VehicleSide side) const {
    if (!m_connected)
        return 0;

    if (axle == m_driven_axles[0]) {
        switch (side) {
            case LEFT:
                return -m_driven_left->GetAppliedTorque();
            case RIGHT:
                return -m_driven_right->GetAppliedTorque();
        }
    }

    return 0;
}

// -----------------------------------------------------------------------------
void Generic_SimpleDriveline::Disconnect() {
    m_connected = false;
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
