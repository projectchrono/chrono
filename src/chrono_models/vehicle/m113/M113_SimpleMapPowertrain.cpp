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
// Simple powertrain model for the M113 vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// Based on data from:  http://powerforce.com/PDFs/2Cycle_Engines/DS_PF6V-53N.pdf
// =============================================================================

#include "chrono_models/vehicle/m113/M113_SimpleMapPowertrain.h"

namespace chrono {
namespace vehicle {
namespace m113 {

const double rpm2rads = CH_C_2PI / 60.0;
const double lbft2Nm = 1.0 / 0.73756;

M113_SimpleMapPowertrain::M113_SimpleMapPowertrain(const std::string& name) : ChSimpleMapPowertrain(name) {}

double M113_SimpleMapPowertrain::GetMaxEngineSpeed() {
    return 3500 * rpm2rads;
}

void M113_SimpleMapPowertrain::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    map0.AddPoint(-100 * rpm2rads, 0 * lbft2Nm);  // to start engine
    map0.AddPoint(500 * rpm2rads, 450 * lbft2Nm);
    map0.AddPoint(1000 * rpm2rads, 450 * lbft2Nm);
    map0.AddPoint(1500 * rpm2rads, 445 * lbft2Nm);
    map0.AddPoint(2000 * rpm2rads, 435 * lbft2Nm);
    map0.AddPoint(2500 * rpm2rads, 410 * lbft2Nm);
    map0.AddPoint(2800 * rpm2rads, 395 * lbft2Nm);
    map0.AddPoint(3000 * rpm2rads, 380 * lbft2Nm);
    map0.AddPoint(3200 * rpm2rads, -100 * lbft2Nm);  // fading out of engine torque

    mapF.AddPoint(-100 * rpm2rads, 0 * lbft2Nm);  // to start engine
    mapF.AddPoint(500 * rpm2rads, 450 * lbft2Nm);
    mapF.AddPoint(1000 * rpm2rads, 450 * lbft2Nm);
    mapF.AddPoint(1500 * rpm2rads, 445 * lbft2Nm);
    mapF.AddPoint(2000 * rpm2rads, 435 * lbft2Nm);
    mapF.AddPoint(2500 * rpm2rads, 410 * lbft2Nm);
    mapF.AddPoint(2800 * rpm2rads, 395 * lbft2Nm);
    mapF.AddPoint(3000 * rpm2rads, 380 * lbft2Nm);
    mapF.AddPoint(3200 * rpm2rads, -100 * lbft2Nm);  // fading out of engine torque
}

void M113_SimpleMapPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.151;

    fwd.push_back(0.240);  // 1st gear;
    fwd.push_back(0.427);  // 2nd gear;
    fwd.push_back(0.685);  // 3rd gear;
    fwd.push_back(0.962);  // 4th gear;
}

void M113_SimpleMapPowertrain::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(750 * rpm2rads, 1500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(750 * rpm2rads, 1500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(750 * rpm2rads, 1500 * rpm2rads));
    shift_bands.push_back(std::pair<double, double>(750 * rpm2rads, 1500 * rpm2rads));
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
