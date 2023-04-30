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
// Authors: Radu Serban, Asher Elmquist, Marcel Offermans
// =============================================================================

#include "chrono_models/vehicle/sedan/Sedan_AutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace sedan {

const double rpm2rads = CH_C_PI / 30;

Sedan_AutomaticTransmissionSimpleMap::Sedan_AutomaticTransmissionSimpleMap(const std::string& name) : ChAutomaticTransmissionSimpleMap(name) {}

void Sedan_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -1.0 / 3.333;

    fwd.push_back(1.0 / 3.778);
    fwd.push_back(1.0 / 2.045);
    fwd.push_back(1.0 / 1.276);
    fwd.push_back(1.0 / 0.941);
    fwd.push_back(1.0 / 0.784);
    fwd.push_back(1.0 / 0.667);
}

void Sedan_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 4000 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1200 * rpm2rads, 4500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1400 * rpm2rads, 4500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1600 * rpm2rads, 4500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1800 * rpm2rads, 4500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(2000 * rpm2rads, 4500 * rpm2rads));
}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
