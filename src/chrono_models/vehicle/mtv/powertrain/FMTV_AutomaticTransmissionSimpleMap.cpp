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
// Automatic transmssion model for the FMTV vehicle.
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#include "chrono_models/vehicle/mtv/powertrain/FMTV_AutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

const double rpm2rads = CH_C_PI / 30;

FMTV_AutomaticTransmissionSimpleMap::FMTV_AutomaticTransmissionSimpleMap(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void FMTV_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    // gear ratios from the original Steyr 12m18 ZF gearbox (handshifted, same as in the MAN 7t?)
    rev = -0.089525515;

    fwd.push_back(0.077160494);
    fwd.push_back(0.11778563);
    fwd.push_back(0.162337662);
    fwd.push_back(0.220750552);
    fwd.push_back(0.283286119);
    fwd.push_back(0.414937759);
    fwd.push_back(0.571428571);
    fwd.push_back(0.78125);
    fwd.push_back(1.0);
}

void FMTV_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2210 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2700 * rpm2rads));
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
