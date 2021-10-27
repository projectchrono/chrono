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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Simple powertrain model for the MAN 5t vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// Original Engine Deutz F8L513F 256 kW, no data avilable, out of production
// Used:    Deutz TCD 2013 L4 4V 181 kW found at https://www.deutz.com/produkte/motoren/
//
//
// =============================================================================

#include "chrono_models/vehicle/man/MAN_7t_SimpleMapPowertrain.h"

namespace chrono {
namespace vehicle {
namespace man {

const double rpm2rads = CH_C_PI / 30;
const double lbft2nm = 1.3558;

MAN_7t_SimpleMapPowertrain::MAN_7t_SimpleMapPowertrain(const std::string& name) : ChSimpleMapPowertrain(name) {}

double MAN_7t_SimpleMapPowertrain::GetMaxEngineSpeed() {
    return 2300 * rpm2rads;
}

void MAN_7t_SimpleMapPowertrain::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    map0.AddPoint(-100 * rpm2rads, 0.000 * lbft2nm);
    map0.AddPoint(0 * rpm2rads, 0.0 * lbft2nm);
    map0.AddPoint(100 * rpm2rads, 0.0 * lbft2nm);
    map0.AddPoint(400 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(600 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(800 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(1000 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(1200 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(1400 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(1600 * rpm2rads, -20.0 * lbft2nm);
    map0.AddPoint(1800 * rpm2rads, -30.0 * lbft2nm);
    map0.AddPoint(2000 * rpm2rads, -30.0 * lbft2nm);
    map0.AddPoint(2100 * rpm2rads, -40.0 * lbft2nm);
    map0.AddPoint(2300 * rpm2rads, -100.0 * lbft2nm);
    map0.AddPoint(2500 * rpm2rads, -150.0 * lbft2nm);

    mapF.AddPoint(-100.0 * rpm2rads, 0.00);
    mapF.AddPoint(0.0 * rpm2rads, 300.00);
    mapF.AddPoint(400.0 * rpm2rads, 500.00);
    mapF.AddPoint(800.0 * rpm2rads, 700.00);
    mapF.AddPoint(981.97 * rpm2rads, 991.67);
    mapF.AddPoint(1030.65 * rpm2rads, 1184.72);
    mapF.AddPoint(1073.93 * rpm2rads, 1305.56);
    mapF.AddPoint(1124.41 * rpm2rads, 1383.33);
    mapF.AddPoint(1198.34 * rpm2rads, 1426.39);
    mapF.AddPoint(1268.65 * rpm2rads, 1440.28);
    mapF.AddPoint(1355.2 * rpm2rads, 1445.83);
    mapF.AddPoint(1459.78 * rpm2rads, 1445.83);
    mapF.AddPoint(1564.36 * rpm2rads, 1447.22);
    mapF.AddPoint(1676.14 * rpm2rads, 1447.22);
    mapF.AddPoint(1777.12 * rpm2rads, 1437.5);
    mapF.AddPoint(1910.54 * rpm2rads, 1401.39);
    mapF.AddPoint(1980.86 * rpm2rads, 1370.83);
    mapF.AddPoint(2065.6 * rpm2rads, 1318.06);
    mapF.AddPoint(2134.12 * rpm2rads, 1261.11);
    mapF.AddPoint(2186.41 * rpm2rads, 1208.33);
    mapF.AddPoint(2231.48 * rpm2rads, 1163.89);
    mapF.AddPoint(2350.00 * rpm2rads, -100.0);
}

void MAN_7t_SimpleMapPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.089525515;

    fwd.push_back(0.077);
    fwd.push_back(0.118);
    fwd.push_back(0.162);
    fwd.push_back(0.221);
    fwd.push_back(0.283);
    fwd.push_back(0.416);
    fwd.push_back(0.573);
    fwd.push_back(0.780);
    fwd.push_back(1.0);
}

void MAN_7t_SimpleMapPowertrain::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2200 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2200 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2200 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2200 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2200 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2200 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2200 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2200 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2200 * rpm2rads));
}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
