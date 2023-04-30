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
// mrole engine model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/mrole/powertrain/mrole_EngineShafts.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// Static variables
const double mrole_EngineShafts::m_motorblock_inertia = 10.5;
const double mrole_EngineShafts::m_motorshaft_inertia = 2.1;

mrole_EngineShafts::mrole_EngineShafts(const std::string& name) : ChEngineShafts(name, ChVector<>(1, 0, 0)) {}

void mrole_EngineShafts::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-100 * rpm_to_radsec, 600);
    map->AddPoint(702.26 * rpm_to_radsec, 700);
    map->AddPoint(1099 * rpm_to_radsec, 2044);
    map->AddPoint(1158 * rpm_to_radsec, 2149);
    map->AddPoint(1207 * rpm_to_radsec, 2250);
    map->AddPoint(1233 * rpm_to_radsec, 2350);
    map->AddPoint(1263 * rpm_to_radsec, 2450);
    map->AddPoint(1300 * rpm_to_radsec, 2545);
    map->AddPoint(1352 * rpm_to_radsec, 2628);
    map->AddPoint(1403 * rpm_to_radsec, 2683);
    map->AddPoint(1499 * rpm_to_radsec, 2702);
    map->AddPoint(1628 * rpm_to_radsec, 2683);
    map->AddPoint(1757 * rpm_to_radsec, 2650);
    map->AddPoint(1901 * rpm_to_radsec, 2569);
    map->AddPoint(2004 * rpm_to_radsec, 2472);
    map->AddPoint(2099 * rpm_to_radsec, 2386);
    map->AddPoint(2195 * rpm_to_radsec, 2298);
    map->AddPoint(2323 * rpm_to_radsec, 2154);
    map->AddPoint(2450 * rpm_to_radsec, -1000.0);  // fading out of engine torque
}

void mrole_EngineShafts::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-50 * rpm_to_radsec, 30);  // it should never work in negative direction, anyway..
    map->AddPoint(0 * rpm_to_radsec, 0);
    map->AddPoint(50 * rpm_to_radsec, -300);
    map->AddPoint(1000 * rpm_to_radsec, -500);
    map->AddPoint(2000 * rpm_to_radsec, -700);
    map->AddPoint(3000 * rpm_to_radsec, -900);
}

}  // end namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
