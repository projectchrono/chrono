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

#include "chrono_models/vehicle/feda/FEDA_AutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace feda {

const double rpm2rads = CH_C_PI / 30;

FEDA_AutomaticTransmissionSimpleMap::FEDA_AutomaticTransmissionSimpleMap(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void FEDA_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -1.0 / 3.81;

    fwd.push_back(1.0 / 3.74);
    fwd.push_back(1.0 / 2.003);
    fwd.push_back(1.0 / 1.343);
    fwd.push_back(1.0 / 1.0);
    fwd.push_back(1.0 / 0.773);
    fwd.push_back(1.0 / 0.634);
}

void FEDA_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1200 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1200 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1200 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1200 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1200 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1200 * rpm2rads, 2700 * rpm2rads));
}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono
