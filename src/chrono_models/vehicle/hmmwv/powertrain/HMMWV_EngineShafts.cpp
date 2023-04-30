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
// HMMWV engine model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/hmmwv/powertrain/HMMWV_EngineShafts.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// Static variables
const double HMMWV_EngineShafts::m_motorblock_inertia = 10.5;
const double HMMWV_EngineShafts::m_motorshaft_inertia = 1.1;

HMMWV_EngineShafts::HMMWV_EngineShafts(const std::string& name) : ChEngineShafts(name, ChVector<>(1, 0, 0)) {}

void HMMWV_EngineShafts::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-100 * rpm_to_radsec, 300);  // to start engine
    map->AddPoint(800 * rpm_to_radsec, 382);
    map->AddPoint(900 * rpm_to_radsec, 490);
    map->AddPoint(1000 * rpm_to_radsec, 579);
    map->AddPoint(1100 * rpm_to_radsec, 650);
    map->AddPoint(1200 * rpm_to_radsec, 706);
    map->AddPoint(1300 * rpm_to_radsec, 746);
    map->AddPoint(1400 * rpm_to_radsec, 774);
    map->AddPoint(1500 * rpm_to_radsec, 789);
    map->AddPoint(1600 * rpm_to_radsec, 793);
    map->AddPoint(1700 * rpm_to_radsec, 788);
    map->AddPoint(1800 * rpm_to_radsec, 774);
    map->AddPoint(1900 * rpm_to_radsec, 754);
    map->AddPoint(2000 * rpm_to_radsec, 728);
    map->AddPoint(2100 * rpm_to_radsec, 697);
    map->AddPoint(2200 * rpm_to_radsec, 664);
    map->AddPoint(2300 * rpm_to_radsec, 628);
    map->AddPoint(2400 * rpm_to_radsec, 593);
    map->AddPoint(2500 * rpm_to_radsec, 558);
    map->AddPoint(2700 * rpm_to_radsec, -400);  // fading out of engine torque
}

void HMMWV_EngineShafts::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-50 * rpm_to_radsec, 30);  // it should never work in negative direction, anyway..
    map->AddPoint(0 * rpm_to_radsec, 0);
    map->AddPoint(50 * rpm_to_radsec, -30);
    map->AddPoint(1000 * rpm_to_radsec, -50);
    map->AddPoint(2000 * rpm_to_radsec, -70);
    map->AddPoint(3000 * rpm_to_radsec, -90);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
