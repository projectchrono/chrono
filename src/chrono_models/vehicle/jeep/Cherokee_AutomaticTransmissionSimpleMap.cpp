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
// Automatic transmssion model for the Jeep Cherokee vehicle.
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
// gear ratios 3 to 6 from automatic gear box (1 to 4)
// gears 1 and 2 shall simulate the torque converter and the low range gears
//
// =============================================================================

#include "Cherokee_AutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace jeep {

const double rpm2rads = CH_PI / 30;

Cherokee_AutomaticTransmissionSimpleMap::Cherokee_AutomaticTransmissionSimpleMap(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void Cherokee_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.1790;

    fwd.push_back(1.0/11.49);
    fwd.push_back(1.0/6.67);
    fwd.push_back(1.0/3.87);
    fwd.push_back(1.0/2.25);
    fwd.push_back(1.0/1.44);
    fwd.push_back(1.0);
}

void Cherokee_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 4000 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 4000 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 4500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 4500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 4500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 5300 * rpm2rads));
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
