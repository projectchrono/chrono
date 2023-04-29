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
// Authors: Radu Serban, Marcel Offermans
// =============================================================================

#include "chrono_models/vehicle/gator/Gator_AutomaticTransmissionSimple.h"

namespace chrono {
namespace vehicle {
namespace gator {

const double rpm2rads = CH_C_PI / 30;

const double fwd_gear_ratio = 0.07;
const double rev_gear_ratio = -0.04;

Gator_AutomaticTransmissionSimple::Gator_AutomaticTransmissionSimple(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void Gator_AutomaticTransmissionSimple::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = rev_gear_ratio;
    fwd.push_back(fwd_gear_ratio);
}

void Gator_AutomaticTransmissionSimple::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {}

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono
