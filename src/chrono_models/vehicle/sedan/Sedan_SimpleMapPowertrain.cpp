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
// Torque RPM map to match 2018 Audi A4
// Reference material:
// https://www.automobile-catalog.com/curve/2018/2601275/audi_a4_2_0_tfsi_252_quattro.html#gsc.tab=0
// https://www.automobile-catalog.com/auta_perf1.php#gsc.tab=0
//
// =============================================================================

#include "chrono_models/vehicle/sedan/Sedan_SimpleMapPowertrain.h"

namespace chrono {
namespace vehicle {
namespace sedan {

const double rpm2rads = CH_C_PI / 30;

Sedan_SimpleMapPowertrain::Sedan_SimpleMapPowertrain(const std::string& name) : ChSimpleMapPowertrain(name) {}

double Sedan_SimpleMapPowertrain::GetMaxEngineSpeed() {
    return 6500 * rpm2rads;
}

void Sedan_SimpleMapPowertrain::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    map0.AddPoint(-10.0, 0.0);
    map0.AddPoint(10.0, 0.0);
    map0.AddPoint(rpm2rads * 1000, -10.0);
    map0.AddPoint(rpm2rads * 1500, -10.0);
    map0.AddPoint(rpm2rads * 2000, -15.0);
    map0.AddPoint(rpm2rads * 2500, -15.0);
    map0.AddPoint(rpm2rads * 3000, -15.0);
    map0.AddPoint(rpm2rads * 3500, -20.0);
    map0.AddPoint(rpm2rads * 4000, -20.0);
    map0.AddPoint(rpm2rads * 4500, -30.0);
    map0.AddPoint(rpm2rads * 5000, -50.0);
    map0.AddPoint(rpm2rads * 6000, -70.0);
    map0.AddPoint(rpm2rads * 6500, -100.0);

    mapF.AddPoint(-10.0, 0.6 * 174.4);
    mapF.AddPoint(rpm2rads * 1000, 236.8);
    mapF.AddPoint(rpm2rads * 1200, 296.0);
    mapF.AddPoint(rpm2rads * 1400, 338.3);
    mapF.AddPoint(rpm2rads * 1500, 355.2);
    mapF.AddPoint(rpm2rads * 1600, 370.0);
    mapF.AddPoint(rpm2rads * 2000, 370.0);
    mapF.AddPoint(rpm2rads * 2500, 370.0);
    mapF.AddPoint(rpm2rads * 3000, 370.0);
    mapF.AddPoint(rpm2rads * 3500, 370.0);
    mapF.AddPoint(rpm2rads * 4000, 370.0);
    mapF.AddPoint(rpm2rads * 4500, 370.0);
    mapF.AddPoint(rpm2rads * 4600, 369.3);
    mapF.AddPoint(rpm2rads * 4800, 364.0);
    mapF.AddPoint(rpm2rads * 5000, 353.3);
    mapF.AddPoint(rpm2rads * 5200, 339.7);
    mapF.AddPoint(rpm2rads * 5500, 321.2);
    mapF.AddPoint(rpm2rads * 5700, 309.9);
    mapF.AddPoint(rpm2rads * 6000, 294.4);
    mapF.AddPoint(rpm2rads * 6200, 280.4);
    mapF.AddPoint(rpm2rads * 6500, 244.6);
}

void Sedan_SimpleMapPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -1.0 / 3.333;

    fwd.push_back(1.0 / 3.778);
    fwd.push_back(1.0 / 2.045);
    fwd.push_back(1.0 / 1.276);
    fwd.push_back(1.0 / 0.941);
    fwd.push_back(1.0 / 0.784);
    fwd.push_back(1.0 / 0.667);
}

void Sedan_SimpleMapPowertrain::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 4000 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1200 * rpm2rads, 4500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1400 * rpm2rads, 4500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1600 * rpm2rads, 4500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1800 * rpm2rads, 4500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(2000 * rpm2rads, 4500 * rpm2rads));
}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
