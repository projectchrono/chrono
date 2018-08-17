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
    return 2700 * rpm2rads;
}

void Sedan_SimpleMapPowertrain::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    map0.AddPoint(-10.472, 0.000);
    map0.AddPoint(83.776, -20.0);

    map0.AddPoint(104.72, -20.0);   //1000 rpm
    map0.AddPoint(157.08, -20.0);   //1500 rpm
    map0.AddPoint(209.44, -20.0);   //2000 rpm
    map0.AddPoint(261.80, -20.0);   //2500 rpm
    map0.AddPoint(314.16, -20.0);   //3000 rpm
    map0.AddPoint(366.52, -20.0);   //3500 rpm
    map0.AddPoint(418.88, -30.0);   //4000 rpm
    map0.AddPoint(471.24, -30.0);   //4500 rpm
    map0.AddPoint(523.60, -40.0);   //5000 rpm
    map0.AddPoint(575.96, -50.0);   //5500 rpm
    map0.AddPoint(628.32, -70.0);   //6000 rpm
    map0.AddPoint(680.68, -100.0);   //6500 rpm

    map0.AddPoint(733.04, -800.0);   //7000 rpm

    mapF.AddPoint(-10.472, 100.);
    mapF.AddPoint(83.776, 112.);

    mapF.AddPoint(104.72, 115.);   //1000 rpm
    mapF.AddPoint(157.08, 170.);   //1500 rpm
    mapF.AddPoint(209.44, 202.);   //2000 rpm
    mapF.AddPoint(261.80, 220.);   //2500 rpm
    mapF.AddPoint(314.16, 230);   //3000 rpm
    mapF.AddPoint(366.52, 240);   //3500 rpm
    mapF.AddPoint(418.88, 245);   //4000 rpm
    mapF.AddPoint(471.24, 242);   //4500 rpm
    mapF.AddPoint(523.60, 240);   //5000 rpm
    mapF.AddPoint(575.96, 228);   //5500 rpm
    mapF.AddPoint(628.32, 218);   //6000 rpm
    mapF.AddPoint(680.68, 200);   //6500 rpm

    mapF.AddPoint(733.04, -800.0);   //7000 rpm
}

void Sedan_SimpleMapPowertrain::SetGearRatios(std::vector<double>& fwd_gear_ratios, double& reverse_gear_ratio) {
    reverse_gear_ratio = -0.272;

    fwd_gear_ratios.push_back(0.274);
    fwd_gear_ratios.push_back(0.481);
    fwd_gear_ratios.push_back(0.735);
    fwd_gear_ratios.push_back(0.977);
    fwd_gear_ratios.push_back(1.205);
    fwd_gear_ratios.push_back(1.458);
}

void Sedan_SimpleMapPowertrain::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2210 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2226 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2225 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(1000 * rpm2rads, 2700 * rpm2rads));
}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
