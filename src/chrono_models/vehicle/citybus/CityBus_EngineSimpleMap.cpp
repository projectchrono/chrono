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
// Authors: Radu Serban, Asher Elmquist, Evan Hoerl, Marcel Offermans
// =============================================================================

#include "chrono_models/vehicle/citybus/CityBus_EngineSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace citybus {

const double rpm2rads = CH_C_PI / 30;
const double lbft2nm = 1.3558;

CityBus_EngineSimpleMap::CityBus_EngineSimpleMap(const std::string& name) : ChEngineSimpleMap(name) {}

double CityBus_EngineSimpleMap::GetMaxEngineSpeed() {
    return 2200 * rpm2rads;
}

void CityBus_EngineSimpleMap::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    map0.AddPoint(-100 * rpm2rads,    0.0 * lbft2nm);
    map0.AddPoint(   0 * rpm2rads,    0.0 * lbft2nm);
    map0.AddPoint( 100 * rpm2rads,    0.0 * lbft2nm);
    map0.AddPoint( 400 * rpm2rads,  -20.0 * lbft2nm);
    map0.AddPoint( 600 * rpm2rads,  -20.0 * lbft2nm);
    map0.AddPoint( 800 * rpm2rads,  -20.0 * lbft2nm);
    map0.AddPoint(1000 * rpm2rads,  -20.0 * lbft2nm);
    map0.AddPoint(1200 * rpm2rads,  -20.0 * lbft2nm);
    map0.AddPoint(1400 * rpm2rads,  -20.0 * lbft2nm);
    map0.AddPoint(1600 * rpm2rads,  -20.0 * lbft2nm);
    map0.AddPoint(1800 * rpm2rads,  -30.0 * lbft2nm);
    map0.AddPoint(2000 * rpm2rads,  -30.0 * lbft2nm);
    map0.AddPoint(2100 * rpm2rads,  -40.0 * lbft2nm);
    map0.AddPoint(2200 * rpm2rads, -100.0 * lbft2nm);

    mapF.AddPoint(-100 * rpm2rads,    0.0 * lbft2nm);
    mapF.AddPoint( 100 * rpm2rads,  300.0 * lbft2nm);
    mapF.AddPoint( 400 * rpm2rads,  500.0 * lbft2nm);
    mapF.AddPoint( 600 * rpm2rads,  600.0 * lbft2nm);
    mapF.AddPoint( 800 * rpm2rads,  800.0 * lbft2nm);
    mapF.AddPoint(1000 * rpm2rads, 1200.0 * lbft2nm);
    mapF.AddPoint(1200 * rpm2rads, 1250.0 * lbft2nm);
    mapF.AddPoint(1400 * rpm2rads, 1250.0 * lbft2nm);
    mapF.AddPoint(1600 * rpm2rads, 1150.0 * lbft2nm);
    mapF.AddPoint(1800 * rpm2rads, 1000.0 * lbft2nm);
    mapF.AddPoint(2000 * rpm2rads,  900.0 * lbft2nm);
    mapF.AddPoint(2100 * rpm2rads,  820.0 * lbft2nm);
    mapF.AddPoint(2200 * rpm2rads, -100.0 * lbft2nm);
}

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
