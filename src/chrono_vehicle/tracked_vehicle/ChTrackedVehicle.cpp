// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for a tracked vehicle system.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Update the state of this vehicle at the current time.
// The vehicle system is provided the current driver inputs (throttle between
// 0 and 1, steering between -1 and +1, braking between 0 and 1), the torque
// from the powertrain, and tire forces (expressed in the global reference
// frame).
// The default implementation of this function invokes the update functions for
// all vehicle subsystems.
// -----------------------------------------------------------------------------
void ChTrackedVehicle::Update(double time,
                              double steering,
                              double braking,
                              double powertrain_torque,
                              const TrackShoeForces& shoe_forces_left,
                              const TrackShoeForces& shoe_forces_right) {
    // Apply powertrain torque to the driveline's input shaft.
    m_driveline->Update(steering, powertrain_torque);

    // Apply contact track shoe forces.
    m_tracks[LEFT]->Update(time, shoe_forces_left);
    m_tracks[RIGHT]->Update(time, shoe_forces_right);

    // Apply braking.
    //// TODO
}

// -----------------------------------------------------------------------------
// Log constraint violations
// -----------------------------------------------------------------------------
void ChTrackedVehicle::LogConstraintViolations() {
    GetLog().SetNumFormat("%16.4e");

    // Report constraint violations for the track assemblies.
    GetLog() << "\n---- LEFT TRACK ASSEMBLY constraint violations\n\n";
    m_tracks[0]->LogConstraintViolations();
    GetLog() << "\n---- RIGHT TRACK ASSEMBLY constraint violations\n\n";
    m_tracks[1]->LogConstraintViolations();

    GetLog().SetNumFormat("%g");
}

}  // end namespace vehicle
}  // end namespace chrono
