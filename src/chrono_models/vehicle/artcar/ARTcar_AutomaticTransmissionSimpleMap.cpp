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
// Authors: Radu Serban, Jayne Henry, Luning Fang, Marcel Offermans
// =============================================================================

#include "chrono_models/vehicle/artcar/ARTcar_AutomaticTransmissionSimpleMap.h"

using namespace chrono::vehicle;
using namespace chrono;

namespace chrono {
namespace vehicle {
namespace artcar {

ARTcar_AutomaticTransmissionSimpleMap::ARTcar_AutomaticTransmissionSimpleMap(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void ARTcar_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -1.0 / 3;
    fwd.push_back(1.0 / 3);
}

void ARTcar_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    // never shifts
}

}  // end namespace artcar
}  // namespace vehicle
}  // namespace chrono
