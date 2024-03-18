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
// Authors: Radu Serban, Asher Elmquist, Marcel Offermans, Rainer Gericke
// =============================================================================

#include "chrono_models/vehicle/bmw/BMW_E90_EngineSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace bmw {

const double rpm2rads = CH_PI / 30;

BMW_E90_EngineSimpleMap::BMW_E90_EngineSimpleMap(const std::string& name) : ChEngineSimpleMap(name) {}

double BMW_E90_EngineSimpleMap::GetMaxEngineSpeed() {
    return 7200 * rpm2rads;
}

void BMW_E90_EngineSimpleMap::SetEngineTorqueMaps(ChFunctionInterp& map0, ChFunctionInterp& mapF) {
    // 225 kW 335i Engine scaled down to 200 kW 330i
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
    map0.AddPoint(rpm2rads * 7200, -100.0);

    mapF.AddPoint(-10.0, 0.6 * 174.4);
    mapF.AddPoint(rpm2rads * 992, 269);
    mapF.AddPoint(rpm2rads * 1433, 359.9);
    mapF.AddPoint(rpm2rads * 5028, 359.9);
    mapF.AddPoint(rpm2rads * 5649, 336.8);
    mapF.AddPoint(rpm2rads * 6000, 318.3);
    mapF.AddPoint(rpm2rads * 6400, 295.2);
    mapF.AddPoint(rpm2rads * 7000, 251.2);
    mapF.AddPoint(rpm2rads * 7200, -100);
}

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono
