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
// Authors: Radu Serban, Asher Elmquist, Evan Hoerl
// =============================================================================
//
// Simple powertrain model for the CityBus vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#include "chrono_models/vehicle/citybus/CityBus_SimpleMapPowertrain.h"

namespace chrono {
namespace vehicle {
namespace citybus {

const double rpm2rads = CH_C_PI / 30;
const double lbft2nm = 1.3558;

CityBus_SimpleMapPowertrain::CityBus_SimpleMapPowertrain(const std::string& name) : ChSimpleMapPowertrain(name) {}

double CityBus_SimpleMapPowertrain::GetMaxEngineSpeed() {
    return 2200 * rpm2rads;
}

void CityBus_SimpleMapPowertrain::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    map0.AddPoint(-100*rpm2rads, 0.000*lbft2nm);
    map0.AddPoint(0*rpm2rads, -20.0*lbft2nm);
    map0.AddPoint(100*rpm2rads, -20.0*lbft2nm);
    map0.AddPoint(400*rpm2rads, -20.0*lbft2nm);
    map0.AddPoint(600*rpm2rads, -20.0*lbft2nm);
    map0.AddPoint(800*rpm2rads, -20.0*lbft2nm);
    map0.AddPoint(1000*rpm2rads, -20.0*lbft2nm);
    map0.AddPoint(1200*rpm2rads, -20.0*lbft2nm);
    map0.AddPoint(1400*rpm2rads, -20.0*lbft2nm);
    map0.AddPoint(1600*rpm2rads, -20.0*lbft2nm);
    map0.AddPoint(1800*rpm2rads, -30.0*lbft2nm);
    map0.AddPoint(2000*rpm2rads, -30.0*lbft2nm);
    map0.AddPoint(2100*rpm2rads, -40.0*lbft2nm);
    map0.AddPoint(2200*rpm2rads, -100.0*lbft2nm);


    mapF.AddPoint(-100*rpm2rads, 0.000*lbft2nm);
    mapF.AddPoint(0*rpm2rads, 300.0*lbft2nm);
    mapF.AddPoint(100*rpm2rads, 400.0*lbft2nm);
    mapF.AddPoint(400*rpm2rads, 500.0*lbft2nm);
    mapF.AddPoint(600*rpm2rads, 600.0*lbft2nm);
    mapF.AddPoint(800*rpm2rads, 800.0*lbft2nm);
    mapF.AddPoint(1000*rpm2rads, 1200.0*lbft2nm);
    mapF.AddPoint(1200*rpm2rads, 1250.0*lbft2nm);
    mapF.AddPoint(1400*rpm2rads, 1250.0*lbft2nm);
    mapF.AddPoint(1600*rpm2rads, 1150.0*lbft2nm);
    mapF.AddPoint(1800*rpm2rads, 1000.0*lbft2nm);
    mapF.AddPoint(2000*rpm2rads, 900.0*lbft2nm);
    mapF.AddPoint(2100*rpm2rads, 820.0*lbft2nm);
    mapF.AddPoint(2200*rpm2rads, -100.0*lbft2nm);
}

void CityBus_SimpleMapPowertrain::SetGearRatios(std::vector<double>& fwd_gear_ratios, double& reverse_gear_ratio) {
    reverse_gear_ratio = -.20;

    fwd_gear_ratios.push_back(.29);
    fwd_gear_ratios.push_back(.54);
    fwd_gear_ratios.push_back(.71);
    fwd_gear_ratios.push_back(1.0);
    fwd_gear_ratios.push_back(1.33);
    fwd_gear_ratios.push_back(1.54);
}

void CityBus_SimpleMapPowertrain::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(800 * rpm2rads, 1500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(800 * rpm2rads, 1500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(800 * rpm2rads, 1500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(800 * rpm2rads, 1500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(800 * rpm2rads, 1500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(800 * rpm2rads, 2200 * rpm2rads));
}

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
