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
// Authors: Radu Serban, Mike Taylor, Marcel Offermans
// =============================================================================

#include "chrono_models/vehicle/generic/powertrain/Generic_AutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace generic {

const double rpm2rads = CH_C_PI / 30;

Generic_AutomaticTransmissionSimpleMap::Generic_AutomaticTransmissionSimpleMap(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void Generic_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.3333;

    fwd.push_back(0.2857);
    fwd.push_back(0.5155);
    fwd.push_back(0.7937);
    fwd.push_back(1.0753);
    fwd.push_back(1.3158);
    fwd.push_back(1.4815);
}

void Generic_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(200, 600));
    shift_bands.push_back(std::pair<double, double>(200, 600));
    shift_bands.push_back(std::pair<double, double>(200, 600));
    shift_bands.push_back(std::pair<double, double>(200, 600));
    shift_bands.push_back(std::pair<double, double>(200, 600));
    shift_bands.push_back(std::pair<double, double>(200, 600));
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
