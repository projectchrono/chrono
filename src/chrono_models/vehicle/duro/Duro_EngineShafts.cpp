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
// Duro engine model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/duro/Duro_EngineShafts.h"

namespace chrono {
namespace vehicle {
namespace duro {

// Static variables
const double Duro_EngineShafts::m_motorblock_inertia = 10.5;
const double Duro_EngineShafts::m_motorshaft_inertia = 1.1;

Duro_EngineShafts::Duro_EngineShafts(const std::string& name) : ChEngineShafts(name, ChVector<>(1, 0, 0)) {}

void Duro_EngineShafts::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
    // could not get data for original 110 kW at 4000 rpm engine (VM Motori 20B/EP638LI)
    // we take this (more modern) 120 kW at 3000 rpm (VM Motori R756IE5/EU6C)
    // Driveline must be adjusted to reach the max. speed of 110 km/h or 55 km/h offroad
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-100 * rpm_to_radsec, 300);  // to start engine
    map->AddPoint(800 * rpm_to_radsec, 398);
    map->AddPoint(1000 * rpm_to_radsec, 438);
    map->AddPoint(1200 * rpm_to_radsec, 478);
    map->AddPoint(1400 * rpm_to_radsec, 500);
    map->AddPoint(1600 * rpm_to_radsec, 491);
    map->AddPoint(1800 * rpm_to_radsec, 478);
    map->AddPoint(2000 * rpm_to_radsec, 462);
    map->AddPoint(2200 * rpm_to_radsec, 447);
    map->AddPoint(2400 * rpm_to_radsec, 431);
    map->AddPoint(2600 * rpm_to_radsec, 414);
    map->AddPoint(2800 * rpm_to_radsec, 392);
    map->AddPoint(3000 * rpm_to_radsec, 360);
    map->AddPoint(3200 * rpm_to_radsec, 150);
    map->AddPoint(3300 * rpm_to_radsec, -400);  // fading out of engine torque
}

void Duro_EngineShafts::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-50 * rpm_to_radsec, 30);  // it should never work in negative direction, anyway..
    map->AddPoint(0 * rpm_to_radsec, 0);
    map->AddPoint(50 * rpm_to_radsec, -30);
    map->AddPoint(1000 * rpm_to_radsec, -50);
    map->AddPoint(2000 * rpm_to_radsec, -70);
    map->AddPoint(3000 * rpm_to_radsec, -90);
    map->AddPoint(4000 * rpm_to_radsec, -120);
}

}  // end namespace Duro
}  // end namespace vehicle
}  // end namespace chrono

