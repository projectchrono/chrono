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
// AIDriver for the Sedan vehicle.
// Uses a maximum front wheel angle of 0.63 rad (about 36 degrees).
// 
// =============================================================================

#include "chrono_models/vehicle/sedan/Sedan_AIDriver.h"

namespace chrono {
namespace vehicle {
namespace sedan {

const double Sedan_AIDriver::m_max_front_angle = 0.63;

Sedan_AIDriver::Sedan_AIDriver(ChVehicle& vehicle) : ChAIDriver(vehicle) {
    SetSpeedControllerGains(0.8, 0, 0);
    SetThresholdThrottle(0.2);
}

double Sedan_AIDriver::CalculateSteering(double front_axle_angle, double rear_axle_angle) {
    return front_axle_angle / m_max_front_angle;
}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
