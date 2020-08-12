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
// Simple powertrain model for the Sedan vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#include "chrono_models/vehicle/sedan/Sedan_SimpleMapPowertrain.h"

namespace chrono {
namespace vehicle {
namespace sedan {

const double rpm2rads = CH_C_PI / 30;

Sedan_SimpleMapPowertrain::Sedan_SimpleMapPowertrain(const std::string& name) : ChSimpleMapPowertrain(name) {}

double Sedan_SimpleMapPowertrain::GetMaxEngineSpeed() {
    return 6000 * rpm2rads;
}

void Sedan_SimpleMapPowertrain::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    map0.AddPoint(-10.0, 0.0);
    map0.AddPoint(10.0, 0.0);
    map0.AddPoint(rpm2rads * 736, -10.0);
    map0.AddPoint(rpm2rads * 987, -10.0);
    map0.AddPoint(rpm2rads * 1500, -15.0);
    map0.AddPoint(rpm2rads * 1980, -15.0);
    map0.AddPoint(rpm2rads * 2348, -15.0);
    map0.AddPoint(rpm2rads * 2737, -20.0);
    map0.AddPoint(rpm2rads * 3189, -20.0);
    map0.AddPoint(rpm2rads * 3684, -30.0);
    map0.AddPoint(rpm2rads * 3908, -100.0);

    mapF.AddPoint(-10.0, 0.6 * 174.4);
    mapF.AddPoint(rpm2rads * 736, 174.4);
    mapF.AddPoint(rpm2rads * 987, 182.7);
    mapF.AddPoint(rpm2rads * 1238, 189.8);
    mapF.AddPoint(rpm2rads * 1500, 195.7);
    mapF.AddPoint(rpm2rads * 1724, 200.5);
    mapF.AddPoint(rpm2rads * 1980, 205.2);
    mapF.AddPoint(rpm2rads * 2188, 207.6);
    mapF.AddPoint(rpm2rads * 2348, 208.1);
    mapF.AddPoint(rpm2rads * 2492, 206.2);
    mapF.AddPoint(rpm2rads * 2737, 206.2);
    mapF.AddPoint(rpm2rads * 2992, 206.2);
    mapF.AddPoint(rpm2rads * 3189, 206.4);
    mapF.AddPoint(rpm2rads * 3487, 203.1);
    mapF.AddPoint(rpm2rads * 3684, 199.5);
    mapF.AddPoint(rpm2rads * 3908, 194.5);
    mapF.AddPoint(rpm2rads * 4100, -100.0);
    mapF.AddPoint(rpm2rads * 4200, -200.0);
}

void Sedan_SimpleMapPowertrain::SetGearRatios(std::vector<double>& fwd_gear_ratios, double& reverse_gear_ratio) {
    reverse_gear_ratio = -1.0 / 5.224;

    fwd_gear_ratios.push_back(1.0 / 4.124 / 1.94);
    fwd_gear_ratios.push_back(1.0 / 4.124);
    fwd_gear_ratios.push_back(1.0 / 2.641);
    fwd_gear_ratios.push_back(1.0 / 1.58);
    fwd_gear_ratios.push_back(1.0);
}

void Sedan_SimpleMapPowertrain::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3000 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3700 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3700 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3700 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 3700 * rpm2rads));
}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
