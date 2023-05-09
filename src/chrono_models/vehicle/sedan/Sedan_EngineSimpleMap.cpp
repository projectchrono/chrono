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
// Authors: Radu Serban, Asher Elmquist, Marcel Offermans
// =============================================================================

#include "chrono_models/vehicle/sedan/Sedan_EngineSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace sedan {

const double rpm2rads = CH_C_PI / 30;

Sedan_EngineSimpleMap::Sedan_EngineSimpleMap(const std::string& name) : ChEngineSimpleMap(name) {}

double Sedan_EngineSimpleMap::GetMaxEngineSpeed() {
    return 6500 * rpm2rads;
}

void Sedan_EngineSimpleMap::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
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

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
