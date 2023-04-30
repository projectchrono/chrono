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
// FMTV engine model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/mtv/powertrain/FMTV_EngineShafts.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// Static variables
const double FMTV_EngineShafts::m_motorblock_inertia = 10.5;
const double FMTV_EngineShafts::m_motorshaft_inertia = 1.1;

FMTV_EngineShafts::FMTV_EngineShafts(const std::string& name) : ChEngineShafts(name, ChVector<>(1, 0, 0)) {}

void FMTV_EngineShafts::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-100.0 * rpm_to_radsec, 200.00);
    map->AddPoint(0 * rpm_to_radsec, 200.00);
    map->AddPoint(500 * rpm_to_radsec, 300);
    map->AddPoint(1000 * rpm_to_radsec, 500);
    map->AddPoint(1200 * rpm_to_radsec, 572);
    map->AddPoint(1400 * rpm_to_radsec, 664);
    map->AddPoint(1600 * rpm_to_radsec, 713);
    map->AddPoint(1800 * rpm_to_radsec, 733);
    map->AddPoint(2000 * rpm_to_radsec, 725);
    map->AddPoint(2100 * rpm_to_radsec, 717);
    map->AddPoint(2200 * rpm_to_radsec, 707);
    map->AddPoint(2400 * rpm_to_radsec, 682);
    map->AddPoint(2700 * rpm_to_radsec, -400);
}

void FMTV_EngineShafts::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-50 * rpm_to_radsec, 30);  // it should never work in negative direction, anyway..
    map->AddPoint(0 * rpm_to_radsec, 0);
    map->AddPoint(50 * rpm_to_radsec, -30);
    map->AddPoint(1000 * rpm_to_radsec, -50);
    map->AddPoint(2000 * rpm_to_radsec, -70);
    map->AddPoint(3000 * rpm_to_radsec, -90);
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
