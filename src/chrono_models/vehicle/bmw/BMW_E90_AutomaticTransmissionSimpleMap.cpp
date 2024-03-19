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
// Authors: Radu Serban, Asher Elmquist, Marcel Offermans, Rainer Gericke
// data from http://www.treffseiten.de/bmw/info/daten_320i_325i_330i_320d_limousine_05_09.pdf
// Gear ratios from automatic gearbox version
// =============================================================================

#include "chrono_models/vehicle/bmw/BMW_E90_AutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace bmw {

const double rpm2rads = CH_PI / 30;

BMW_E90_AutomaticTransmissionSimpleMap::BMW_E90_AutomaticTransmissionSimpleMap(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void BMW_E90_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -1.0 / 3.4;

    fwd.push_back(1.0 / 4.71);
    fwd.push_back(1.0 / 2.34);
    fwd.push_back(1.0 / 1.52);
    fwd.push_back(1.0 / 1.14);
    fwd.push_back(1.0 / 0.87);
    fwd.push_back(1.0 / 0.69);
}

void BMW_E90_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 5000 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1200 * rpm2rads, 5500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1400 * rpm2rads, 5500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1600 * rpm2rads, 5500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1800 * rpm2rads, 5500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(2000 * rpm2rads, 5500 * rpm2rads));
}

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono
