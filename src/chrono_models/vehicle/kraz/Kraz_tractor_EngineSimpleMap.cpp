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
// Simple EngineSimpleMap model for the Kraz tractor vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#include "chrono_models/vehicle/kraz/Kraz_tractor_EngineSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace kraz {

const double rpm2rads = CH_C_PI / 30;

Kraz_tractor_EngineSimpleMap::Kraz_tractor_EngineSimpleMap(const std::string& name) : ChEngineSimpleMap(name) {}

double Kraz_tractor_EngineSimpleMap::GetMaxEngineSpeed() {
    return 2700 * rpm2rads;
}

void Kraz_tractor_EngineSimpleMap::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
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

    // Original: Caterpillar 3116 Diesel 171 kW
    // we provide some chip tuning to get 272 kW (370 HP) of the Kraz 64431 V8 Diesel
    const double tune = 1.587;
    mapF.AddPoint(-10.472, 406.7 * tune);
    mapF.AddPoint(500 * rpm2rads, 400 * tune);
    mapF.AddPoint(1000 * rpm2rads, 500 * tune);
    mapF.AddPoint(1200 * rpm2rads, 572 * tune);
    mapF.AddPoint(1400 * rpm2rads, 664 * tune);
    mapF.AddPoint(1600 * rpm2rads, 713 * tune);
    mapF.AddPoint(1800 * rpm2rads, 733 * tune);
    mapF.AddPoint(2000 * rpm2rads, 725 * tune);
    mapF.AddPoint(2100 * rpm2rads, 717 * tune);
    mapF.AddPoint(2200 * rpm2rads, 707 * tune);
    mapF.AddPoint(2300 * rpm2rads, 682 * tune);
    mapF.AddPoint(2500 * rpm2rads, -271.2 * tune);
    mapF.AddPoint(2400 * rpm2rads, -800.0 * tune);
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
