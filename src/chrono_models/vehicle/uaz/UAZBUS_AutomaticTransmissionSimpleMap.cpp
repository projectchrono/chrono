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

#include "chrono_models/vehicle/uaz/UAZBUS_AutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace uaz {

const double rpm2rads = CH_C_PI / 30;

UAZBUS_AutomaticTransmissionSimpleMap::UAZBUS_AutomaticTransmissionSimpleMap(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void UAZBUS_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -1.0 / 5.224;

    fwd.push_back(1.0 / 4.124 / 1.94);
    fwd.push_back(1.0 / 4.124);
    fwd.push_back(1.0 / 2.641);
    fwd.push_back(1.0 / 1.58);
    fwd.push_back(1.0);
}

void UAZBUS_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3000 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3700 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3700 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3700 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3700 * rpm2rads));
}

}  // namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
