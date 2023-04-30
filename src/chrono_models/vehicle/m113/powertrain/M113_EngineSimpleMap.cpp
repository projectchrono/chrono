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
// Simple engine model for the M113 vehicle based on torque-speed engine maps
//
// =============================================================================

#include "chrono_models/vehicle/m113/powertrain/M113_EngineSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace m113 {

const double rpm2rads = CH_C_PI / 30;

M113_EngineSimpleMap::M113_EngineSimpleMap(const std::string& name) : ChEngineSimpleMap(name) {}

double M113_EngineSimpleMap::GetMaxEngineSpeed() {
    return 3000 * rpm2rads;
}

void M113_EngineSimpleMap::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    const double lbft2Nm = 1.0 / 0.73756;
    
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

}  // end namespace M113
}  // end namespace vehicle
}  // end namespace chrono

