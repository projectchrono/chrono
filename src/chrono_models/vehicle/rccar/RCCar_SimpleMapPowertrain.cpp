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
// Authors: Radu Serban, Jayne Henry
// =============================================================================
//
// Simple powertrain model for the RCCar vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#include "chrono_models/vehicle/rccar/RCCar_SimpleMapPowertrain.h"

using namespace chrono::vehicle;
using namespace chrono;

namespace chrono {
namespace vehicle {
namespace rccar {

const double rpm2rads = CH_C_PI / 30;

const double max_rpm = 19240;

RCCar_SimpleMapPowertrain::RCCar_SimpleMapPowertrain(const std::string& name) : ChSimpleMapPowertrain(name) {}

double RCCar_SimpleMapPowertrain::GetMaxEngineSpeed() {
    return max_rpm * rpm2rads;
}

void RCCar_SimpleMapPowertrain::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    double m = -2e-3;

    double x1 = max_rpm * rpm2rads, y1 = 0;
    double x0 = 0, y0 = -m * x1;
    int r = 5;
    for (int i = 0; i <= r; i++) {
        double t = (double)i / r;
        double x = (1.0 - t) * x0 + t * x1;
        double y = (1.0 - t) * y0 + t * y1;
        mapF.AddPoint(x, y);
    }

    // N-m and rad/s
    map0.AddPoint(x0, 0);
    map0.AddPoint(x1, 0);
}

void RCCar_SimpleMapPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.2;

    fwd.push_back(1.0 / 3);
}

void RCCar_SimpleMapPowertrain::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(0 * rpm2rads, 100000 * rpm2rads));
}

}  // end namespace rccar
}  // namespace vehicle
}  // namespace chrono
