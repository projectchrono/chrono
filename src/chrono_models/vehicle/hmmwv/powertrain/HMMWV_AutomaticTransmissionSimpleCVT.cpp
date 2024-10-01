// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// Automatic Continuous Variable Transmission (CVT) model for the HMMWV vehicle.
// - both power and torque limited
// - no torque converter
// - simple gear-varying model (linearly dependend on driveshaft angular velocity)
// - simple optional consideration of efficiency
//
// =============================================================================

#include "HMMWV_AutomaticTransmissionSimpleCVT.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

HMMWV_AutomaticTransmissionSimpleCVT::HMMWV_AutomaticTransmissionSimpleCVT(const std::string& name)
    : ChAutomaticTransmissionSimpleCVT(name) {
    // estimate CVT parameters from known values
    double r_tire = 0.42;                                 // static tire radius
    double drl_ratio = 0.2;                               // conical gear ratio from driveline
    double v_start = 2.0;                                 // vehicle speed where gear ratio variation begins
    double v_end = 20.0;                                  // vehicle speed where gear ration variation ends
    double omega_start = v_start / (drl_ratio * r_tire);  // get driveshaft angular velocity from v_start
    double omega_end = v_end / (drl_ratio * r_tire);      // get driveshaft angular velocity from v_end
    double ratio_start = 0.1708;                          // first gear ratio from simple map transmission
    double ratio_end = 1.5361;                            // last gear ratio from simple map transmission
    double eff = 0.8;  // choose cvt transmission efficiency, typical range [0.8 ... 0.86]
    SetOperationRange(omega_start, ratio_start, omega_end, ratio_end, eff);
}

}  // namespace hmmwv
}  // namespace vehicle
}  // namespace chrono