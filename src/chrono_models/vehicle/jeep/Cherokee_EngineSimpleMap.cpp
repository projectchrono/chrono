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
// Authors: Radu Serban
// =============================================================================
//
// Simple engine model for the HMMWV vehicle based on torque-speed engine maps
//
// =============================================================================

#include "chrono_models/vehicle/jeep/Cherokee_EngineSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace jeep {

const double rpm2rads = CH_PI / 30;

Cherokee_EngineSimpleMap::Cherokee_EngineSimpleMap(const std::string& name) : ChEngineSimpleMap(name) {}
double Cherokee_EngineSimpleMap::GetMaxEngineSpeed() {
    return 5300 * rpm2rads;
}

void Cherokee_EngineSimpleMap::SetEngineTorqueMaps(ChFunctionInterp& map0, ChFunctionInterp& mapF) {
    map0.AddPoint(-100 * rpm2rads, 0.000);
    map0.AddPoint(1000 * rpm2rads, -20.0);
    map0.AddPoint(3000 * rpm2rads, -20.0);
    map0.AddPoint(4000*rpm2rads, -20.0);
    map0.AddPoint(5000*rpm2rads, -30.0);
    map0.AddPoint(5300*rpm2rads, -50.0);
    map0.AddPoint(5400*rpm2rads, -500.0);

    mapF.AddPoint(-100 * rpm2rads, 92.3);
    mapF.AddPoint(1000 * rpm2rads, 92.3);
    mapF.AddPoint(1500 * rpm2rads, 179.8);
    mapF.AddPoint(2000 * rpm2rads, 249.4);
    mapF.AddPoint(2500 * rpm2rads, 295.6);
    mapF.AddPoint(3000 * rpm2rads, 315.5);
    mapF.AddPoint(3500 * rpm2rads, 316.2);
    mapF.AddPoint(4000 * rpm2rads, 310.7);
    mapF.AddPoint(4500 * rpm2rads, 296.9);
    mapF.AddPoint(5000 * rpm2rads, 261.8);
    mapF.AddPoint(5300 * rpm2rads, 219.8);
    mapF.AddPoint(5400 * rpm2rads, -400.0);
}

}  // namespace jeep
}  // end namespace vehicle
}  // end namespace chrono
