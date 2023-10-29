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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Simple automatic transmission to use EngineSimpleMap.
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#include "chrono_models/vehicle/gclass/G500_AutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace gclass {

const double rpm2rads = CH_C_PI / 30;

G500_AutomaticTransmissionSimpleMap::G500_AutomaticTransmissionSimpleMap(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void G500_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -1.0 / 8.0;

    fwd.push_back(1.0 / 8.92);
    fwd.push_back(1.0 / 5.4);
    fwd.push_back(1.0 / 3.75);
    fwd.push_back(1.0 / 2.73);
    fwd.push_back(1.0 / 2.02);
    fwd.push_back(1.0 / 1.67);
    fwd.push_back(1.0);
}

void G500_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1200 * rpm2rads, 4000 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1200 * rpm2rads, 5500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1200 * rpm2rads, 5500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1200 * rpm2rads, 5500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1200 * rpm2rads, 5500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1200 * rpm2rads, 5500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1200 * rpm2rads, 5700 * rpm2rads));
}

}  // namespace uaz
}  // end namespace vehicle
}  // end namespace chrono

