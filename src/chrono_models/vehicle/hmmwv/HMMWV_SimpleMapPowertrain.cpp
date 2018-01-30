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
// Simple powertrain model for the HMMWV vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#include "chrono_models/vehicle/hmmwv/HMMWV_SimpleMapPowertrain.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

const double rpm2rads = CH_C_PI / 30;

HMMWV_SimpleMapPowertrain::HMMWV_SimpleMapPowertrain(const std::string& name) : ChSimpleMapPowertrain(name) {}

double HMMWV_SimpleMapPowertrain::GetMaxEngineSpeed() {
    return 2700 * rpm2rads;
}

void HMMWV_SimpleMapPowertrain::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    map0.AddPoint(-10.472, 0.000);
    map0.AddPoint(83.776, -20.0);
    map0.AddPoint(104.720, -20.0);
    map0.AddPoint(125.664, -30.0);
    map0.AddPoint(146.608, -30.0);
    map0.AddPoint(167.552, -30.0);
    map0.AddPoint(188.496, -40.0);
    map0.AddPoint(209.440, -50.0);
    map0.AddPoint(230.383, -70.0);
    map0.AddPoint(251.327, -100.0);
    map0.AddPoint(282.743, -800.0);

    mapF.AddPoint(-10.472, 406.7);
    mapF.AddPoint(83.776, 517.9);
    mapF.AddPoint(104.720, 926.0);
    mapF.AddPoint(125.664, 1216.2);
    mapF.AddPoint(146.608, 1300.2);
    mapF.AddPoint(167.552, 1300.2);
    mapF.AddPoint(188.496, 1227.0);
    mapF.AddPoint(209.440, 1136.2);
    mapF.AddPoint(230.383, 1041.3);
    mapF.AddPoint(251.327, -271.2);
    mapF.AddPoint(282.743, -800.0);
}

void HMMWV_SimpleMapPowertrain::SetGearRatios(std::vector<double>& fwd_gear_ratios, double& reverse_gear_ratio) {
    reverse_gear_ratio = -0.2;

    fwd_gear_ratios.push_back(0.1708);
    fwd_gear_ratios.push_back(0.2791);
    fwd_gear_ratios.push_back(0.4218);
    fwd_gear_ratios.push_back(0.6223);
    fwd_gear_ratios.push_back(1.0173);
    fwd_gear_ratios.push_back(1.5361);
}

void HMMWV_SimpleMapPowertrain::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2210 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2700 * rpm2rads));
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
