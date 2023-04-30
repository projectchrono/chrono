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
// Automatic transmssion model for the Marder vehicle.
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#include "chrono_models/vehicle/marder/powertrain/Marder_AutomaticTransmissionSimple.h"

namespace chrono {
namespace vehicle {
namespace marder {

const double rpm2rads = CH_C_PI / 30;

Marder_AutomaticTransmissionSimple::Marder_AutomaticTransmissionSimple(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void Marder_AutomaticTransmissionSimple::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.2;

    fwd.push_back(0.1708);
}

void Marder_AutomaticTransmissionSimple::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {}

}  // end namespace Marder
}  // end namespace vehicle
}  // end namespace chrono

