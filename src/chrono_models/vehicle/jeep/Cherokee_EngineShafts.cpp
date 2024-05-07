// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// Jeep Cherokee 4WD engine model based on ChShaft objects.
// Engine data taken from:
// https://www.automobile-catalog.com/curve/2006/1317620/jeep_wrangler_sport_4_0l.html#gsc.tab=0
//
// =============================================================================

#include "Cherokee_EngineShafts.h"

namespace chrono {
namespace vehicle {
namespace jeep {

// Static variables
const double Cherokee_EngineShafts::m_motorblock_inertia = 10.5;
const double Cherokee_EngineShafts::m_motorshaft_inertia = 1.1;

Cherokee_EngineShafts::Cherokee_EngineShafts(const std::string& name) : ChEngineShafts(name, ChVector3d(1, 0, 0)) {}

void Cherokee_EngineShafts::SetEngineTorqueMap(std::shared_ptr<ChFunctionInterp>& map) {
    double rpm_to_radsec = CH_2PI / 60.;

    map->AddPoint(-100 * rpm_to_radsec, 90);  // to start engine
    map->AddPoint(1000 * rpm_to_radsec, 92.3);
    map->AddPoint(1500 * rpm_to_radsec, 179.8);
    map->AddPoint(2000 * rpm_to_radsec, 249.4);
    map->AddPoint(2500 * rpm_to_radsec, 295.6);
    map->AddPoint(3000 * rpm_to_radsec, 315.5);
    map->AddPoint(3500 * rpm_to_radsec, 316.2);
    map->AddPoint(4000 * rpm_to_radsec, 310.7);
    map->AddPoint(4500 * rpm_to_radsec, 296.9);
    map->AddPoint(5000 * rpm_to_radsec, 261.8);
    map->AddPoint(5300 * rpm_to_radsec, 219.8);
    map->AddPoint(5600 * rpm_to_radsec, -400);
}

void Cherokee_EngineShafts::SetEngineLossesMap(std::shared_ptr<ChFunctionInterp>& map) {
    double rpm_to_radsec = CH_2PI / 60.;

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
