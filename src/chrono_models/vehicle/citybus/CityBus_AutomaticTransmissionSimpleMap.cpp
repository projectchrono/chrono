// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Asher Elmquist, Evan Hoerl, Marcel Offermans
// =============================================================================

#include "chrono_models/vehicle/citybus/CityBus_AutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace citybus {

const double rpm2rads = CH_C_PI / 30;
const double lbft2nm = 1.3558;

CityBus_AutomaticTransmissionSimpleMap::CityBus_AutomaticTransmissionSimpleMap(const std::string& name) : ChAutomaticTransmissionSimpleMap(name) {}

void CityBus_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.20;
    fwd.push_back(0.29);
    fwd.push_back(0.54);
    fwd.push_back(0.71);
    fwd.push_back(1.0 );
    fwd.push_back(1.33);
    fwd.push_back(1.54);
}

void CityBus_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(800 * rpm2rads, 1500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(800 * rpm2rads, 1500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(800 * rpm2rads, 1500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(800 * rpm2rads, 1500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(800 * rpm2rads, 1500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(800 * rpm2rads, 2200 * rpm2rads));
}

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
