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

#include "chrono_models/vehicle/mtv/powertrain/FMTV_AutomaticTransmissionSimple.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

const double rpm2rads = CH_C_PI / 30;

FMTV_AutomaticTransmissionSimple::FMTV_AutomaticTransmissionSimple(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void FMTV_AutomaticTransmissionSimple::SetGearRatios(std::vector<double>& fwd, double& rev) {
    // gear ratios from the original Steyr 12m18 ZF gearbox (handshifted, same as in the MAN 7t?)
    rev = -0.089525515;

    fwd.push_back(0.077160494);
}

void FMTV_AutomaticTransmissionSimple::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
