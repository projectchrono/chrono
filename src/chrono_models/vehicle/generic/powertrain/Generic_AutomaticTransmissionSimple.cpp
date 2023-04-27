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

#include "chrono_models/vehicle/generic/powertrain/Generic_AutomaticTransmissionSimple.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
Generic_AutomaticTransmissionSimple::Generic_AutomaticTransmissionSimple(const std::string& name) : ChAutomaticTransmissionSimpleMap(name) {}

void Generic_AutomaticTransmissionSimple::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.3;
    fwd.push_back(0.3);
}

void Generic_AutomaticTransmissionSimple::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
