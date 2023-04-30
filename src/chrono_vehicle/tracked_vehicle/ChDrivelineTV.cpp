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
// Base class for a tracked vehicle driveline.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChDrivelineTV.h"

namespace chrono {
namespace vehicle {

ChDrivelineTV::ChDrivelineTV(const std::string& name) : ChDriveline(name), m_gyration_mode(false) {}

void ChDrivelineTV::LockDifferential(bool lock) {
    GetLog() << "WARNING: Differential locking not yet implemented for " << GetTemplateName() << "\n";
}

void ChDrivelineTV::CombineDriverInputs(const DriverInputs& driver_inputs,
                                        double& braking_left,
                                        double& braking_right) {
    braking_left = driver_inputs.m_braking;
    braking_right = driver_inputs.m_braking;
}

}  // end namespace vehicle
}  // end namespace chrono
