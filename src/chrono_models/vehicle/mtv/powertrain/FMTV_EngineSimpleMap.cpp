
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
// Simple engine model for the FMTV vehicle based on torque-speed engine maps
//
// =============================================================================

#include "chrono_models/vehicle/mtv/powertrain/FMTV_EngineSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

const double rpm2rads = CH_C_PI / 30;

FMTV_EngineSimpleMap::FMTV_EngineSimpleMap(const std::string& name) : ChEngineSimpleMap(name) {}

double FMTV_EngineSimpleMap::GetMaxEngineSpeed() {
    return 2700 * rpm2rads;
}

void FMTV_EngineSimpleMap::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
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

    // Caterpillar 3116 Diesel
    mapF.AddPoint(-10.472, 406.7);
    mapF.AddPoint(500 * rpm2rads, 300);
    mapF.AddPoint(1000 * rpm2rads, 500);
    mapF.AddPoint(1200 * rpm2rads, 572);
    mapF.AddPoint(1400 * rpm2rads, 664);
    mapF.AddPoint(1600 * rpm2rads, 713);
    mapF.AddPoint(1800 * rpm2rads, 733);
    mapF.AddPoint(2000 * rpm2rads, 725);
    mapF.AddPoint(2100 * rpm2rads, 717);
    mapF.AddPoint(2200 * rpm2rads, 707);
    mapF.AddPoint(2300 * rpm2rads, 682);
    mapF.AddPoint(2500 * rpm2rads, -271.2);
    mapF.AddPoint(2400 * rpm2rads, -800.0);
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
