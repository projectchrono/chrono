// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Simple engine model for the UAZBUS vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
//
// =============================================================================

#include "chrono_models/vehicle/gclass/G500_EngineSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace gclass {

const double rpm2rads = CH_C_PI / 30;

G500_EngineSimpleMap::G500_EngineSimpleMap(const std::string& name) : ChEngineSimpleMap(name) {}

double G500_EngineSimpleMap::GetMaxEngineSpeed() {
    return 5900 * rpm2rads;
}

void G500_EngineSimpleMap::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    // Mercedes 4.0 litre V8 (M 176) 310kW
    map0.AddPoint(-10.0, 0.0);
    map0.AddPoint(10.0, 0.0);
    map0.AddPoint(rpm2rads * 1000.0, -10.0);
    map0.AddPoint(rpm2rads * 1500.0, -10.0);
    map0.AddPoint(rpm2rads * 1800.0, -15.0);
    map0.AddPoint(rpm2rads * 2250.0, -15.0);
    map0.AddPoint(rpm2rads * 4750.0, -15.0);
    map0.AddPoint(rpm2rads * 5250.0, -20.0);
    map0.AddPoint(rpm2rads * 5500.0, -20.0);
    map0.AddPoint(rpm2rads * 5800.0, -20.0);
    map0.AddPoint(rpm2rads * 5900.0, -100.0);
    map0.AddPoint(rpm2rads * 5950.0, -200.0);

    mapF.AddPoint(-10.0, 0.6 * 400.0);
    mapF.AddPoint(rpm2rads * 1000.0, 400.0);
    mapF.AddPoint(rpm2rads * 1500.0, 500.0);
    mapF.AddPoint(rpm2rads * 1800.0, 550.0);
    mapF.AddPoint(rpm2rads * 2250.0, 610.0);
    mapF.AddPoint(rpm2rads * 4750.0, 610.0);
    mapF.AddPoint(rpm2rads * 5250.0, 565.0);
    mapF.AddPoint(rpm2rads * 5500.0, 539.0);
    mapF.AddPoint(rpm2rads * 5800.0, 450.0);
    mapF.AddPoint(rpm2rads * 5900.0, -100.0);
    mapF.AddPoint(rpm2rads * 5950.0, -200.0);
}

}  // namespace uaz
}  // end namespace vehicle
}  // end namespace chrono

