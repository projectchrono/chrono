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
// M113 engine model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/m113/powertrain/M113_EngineShafts.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// Static variables
const double M113_EngineShafts::m_motorblock_inertia = 10.5;
const double M113_EngineShafts::m_motorshaft_inertia = 1.1;

M113_EngineShafts::M113_EngineShafts(const std::string& name) : ChEngineShafts(name, ChVector<>(1, 0, 0)) {}

void M113_EngineShafts::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.0;
    double lbft_to_Nm = 1.0 / 0.73756;
    
    map->AddPoint(-100 * rpm_to_radsec, 300 * lbft_to_Nm);  // to start engine
    map->AddPoint(500 * rpm_to_radsec, 450 * lbft_to_Nm);
    map->AddPoint(1000 * rpm_to_radsec, 450 * lbft_to_Nm);
    map->AddPoint(1500 * rpm_to_radsec, 445 * lbft_to_Nm);
    map->AddPoint(2000 * rpm_to_radsec, 435 * lbft_to_Nm);
    map->AddPoint(2500 * rpm_to_radsec, 410 * lbft_to_Nm);
    map->AddPoint(2800 * rpm_to_radsec, 395 * lbft_to_Nm);
    map->AddPoint(3000 * rpm_to_radsec, 380 * lbft_to_Nm);
    map->AddPoint(3200 * rpm_to_radsec, -100 * lbft_to_Nm);  // fading out of engine torque
}

void M113_EngineShafts::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-50 * rpm_to_radsec, 30);  // it should never work in negative direction, anyway..
    map->AddPoint(0 * rpm_to_radsec, 0);
    map->AddPoint(50 * rpm_to_radsec, -30);
    map->AddPoint(1000 * rpm_to_radsec, -50);
    map->AddPoint(2000 * rpm_to_radsec, -70);
    map->AddPoint(3000 * rpm_to_radsec, -90);
}

}  // end namespace M113
}  // end namespace vehicle
}  // end namespace chrono

