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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// Simple powertrain model for the FEDA vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#include "chrono_models/vehicle/feda/FEDA_SimpleMapPowertrain.h"

namespace chrono {
namespace vehicle {
namespace feda {

const double rpm2rads = CH_C_PI / 30;

FEDA_SimpleMapPowertrain::FEDA_SimpleMapPowertrain(const std::string& name) : ChSimpleMapPowertrain(name) {}

double FEDA_SimpleMapPowertrain::GetMaxEngineSpeed() {
    return 2550 * rpm2rads;
}

void FEDA_SimpleMapPowertrain::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    const double limit_factor = 0.9;

    map0.AddPoint(-10.472, 0.000);
    map0.AddPoint(83.776, 0.0);

    map0.AddPoint(700 * rpm2rads, -40.0);
    map0.AddPoint(800 * rpm2rads, -41.0);
    map0.AddPoint(900 * rpm2rads, -43.0);
    map0.AddPoint(1000 * rpm2rads, -44.0);
    map0.AddPoint(1100 * rpm2rads, -46.1);
    map0.AddPoint(1200 * rpm2rads, -48.7);
    map0.AddPoint(1300 * rpm2rads, -51.1);
    map0.AddPoint(1400 * rpm2rads, -55.2);
    map0.AddPoint(1500 * rpm2rads, -60.5);
    map0.AddPoint(1600 * rpm2rads, -64.5);
    map0.AddPoint(1700 * rpm2rads, -68.7);
    map0.AddPoint(1800 * rpm2rads, -71.0);
    map0.AddPoint(1900 * rpm2rads, -73.4);
    map0.AddPoint(2000 * rpm2rads, -76.8);
    map0.AddPoint(2100 * rpm2rads, -80.8);
    map0.AddPoint(2200 * rpm2rads, -85.5);
    map0.AddPoint(2300 * rpm2rads, -89.7);
    map0.AddPoint(2400 * rpm2rads, -94.6);
    map0.AddPoint(2500 * rpm2rads, -95.8);
    map0.AddPoint(2525 * rpm2rads, -95.9);
    map0.AddPoint(2850 * rpm2rads, -99.9);

    mapF.AddPoint(-10.472, 300.0 * limit_factor);
    mapF.AddPoint(0, 300.0 * limit_factor);
    mapF.AddPoint(700 * rpm2rads, 400 * limit_factor);
    mapF.AddPoint(800 * rpm2rads, 410 * limit_factor);
    mapF.AddPoint(900 * rpm2rads, 450 * limit_factor);
    mapF.AddPoint(1000 * rpm2rads, 550 * limit_factor);
    mapF.AddPoint(1100 * rpm2rads, 625 * limit_factor);
    mapF.AddPoint(1200 * rpm2rads, 700 * limit_factor);
    mapF.AddPoint(1300 * rpm2rads, 700 * limit_factor);
    mapF.AddPoint(1400 * rpm2rads, 700 * limit_factor);
    mapF.AddPoint(1500 * rpm2rads, 700 * limit_factor);
    mapF.AddPoint(1600 * rpm2rads, 700 * limit_factor);
    mapF.AddPoint(1700 * rpm2rads, 698 * limit_factor);
    mapF.AddPoint(1800 * rpm2rads, 695 * limit_factor);
    mapF.AddPoint(1900 * rpm2rads, 669 * limit_factor);
    mapF.AddPoint(2000 * rpm2rads, 639 * limit_factor);
    mapF.AddPoint(2100 * rpm2rads, 612 * limit_factor);
    mapF.AddPoint(2200 * rpm2rads, 586 * limit_factor);
    mapF.AddPoint(2300 * rpm2rads, 562 * limit_factor);
    mapF.AddPoint(2400 * rpm2rads, 540 * limit_factor);
    mapF.AddPoint(2500 * rpm2rads, 520 * limit_factor);
    mapF.AddPoint(2525 * rpm2rads, 505 * limit_factor);
    mapF.AddPoint(2550 * rpm2rads, 0);
}

void FEDA_SimpleMapPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -1.0 / 3.81;

    fwd.push_back(1.0 / 3.74);
    fwd.push_back(1.0 / 2.003);
    fwd.push_back(1.0 / 1.343);
    fwd.push_back(1.0 / 1.0);
    fwd.push_back(1.0 / 0.773);
    fwd.push_back(1.0 / 0.634);
}

void FEDA_SimpleMapPowertrain::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
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
