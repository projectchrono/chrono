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
// Automatic transmssion model for the M113 vehicle.
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#include "chrono_models/vehicle/m113/powertrain/M113_AutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace m113 {

const double rpm2rads = CH_C_PI / 30;

M113_AutomaticTransmissionSimpleMap::M113_AutomaticTransmissionSimpleMap(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void M113_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.151;

    fwd.push_back(0.240);  // 1st gear;
    fwd.push_back(0.427);  // 2nd gear;
    fwd.push_back(0.685);  // 3rd gear;
    fwd.push_back(0.962);  // 4th gear;
}

void M113_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(750 * rpm2rads, 1500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(750 * rpm2rads, 1500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(750 * rpm2rads, 1500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(750 * rpm2rads, 1500 * rpm2rads));
}

}  // end namespace M113
}  // end namespace vehicle
}  // end namespace chrono

