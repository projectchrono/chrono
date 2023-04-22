// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Duro powertrain model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/duro/Duro_Powertrain.h"

namespace chrono {
namespace vehicle {
namespace duro {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Duro_Powertrain::m_motorblock_inertia = 10.5;
const double Duro_Powertrain::m_crankshaft_inertia = 1.1;
const double Duro_Powertrain::m_ingear_shaft_inertia = 0.3;
const double Duro_Powertrain::m_power_shaft_inertia = 0.5;

const double Duro_Powertrain::m_upshift_RPM = 2800;
const double Duro_Powertrain::m_downshift_RPM = 1200;

// -----------------------------------------------------------------------------
// Constructor of the HMMW_Powertrain.
// the direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
Duro_Powertrain::Duro_Powertrain(const std::string& name) : ChShaftsPowertrain(name, ChVector<>(1, 0, 0)) {
    SetGearShiftLatency(1.0);
}

// -----------------------------------------------------------------------------
// Initialize vector of gear ratios
// -----------------------------------------------------------------------------
void Duro_Powertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    // automatic gearbox Mercedes Benz W 4 A 028
    rev = -1.0 / 5.586;  // reverse gear;

    fwd.push_back(1.0 / 3.871);  // 1st gear;
    fwd.push_back(1.0 / 2.247);  // 2nd gear;
    fwd.push_back(1.0 / 1.436);  // 3rd gear;
    fwd.push_back(1.0);          // 4th gear;
}

// -----------------------------------------------------------------------------
// Set the engine and torque converter maps:
//
// (1) engine speed [rad/s] - torque [Nm] map
//     must be defined beyond max speed too - engine might be 'pulled'
//
// (2) TC capacity factor map
//
// (3) TC torque ratio map
//
// -----------------------------------------------------------------------------
void Duro_Powertrain::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
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

void Duro_Powertrain::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-50 * rpm_to_radsec, 30);  // it should never work in negative direction, anyway..
    map->AddPoint(0 * rpm_to_radsec, 0);
    map->AddPoint(50 * rpm_to_radsec, -30);
    map->AddPoint(1000 * rpm_to_radsec, -50);
    map->AddPoint(2000 * rpm_to_radsec, -70);
    map->AddPoint(3000 * rpm_to_radsec, -90);
    map->AddPoint(4000 * rpm_to_radsec, -120);
}

void Duro_Powertrain::SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) {
    map->AddPoint(0, 6.55649);
    map->AddPoint(0.105301, 6.53588);
    map->AddPoint(0.20916, 6.55649);
    map->AddPoint(0.320936, 6.63897);
    map->AddPoint(0.435681, 6.74205);
    map->AddPoint(0.556364, 6.88638);
    map->AddPoint(0.647369, 6.96885);
    map->AddPoint(0.724529, 7.09256);
    map->AddPoint(0.786849, 7.17503);
    map->AddPoint(0.843236, 7.27812);
    map->AddPoint(0.891714, 7.44306);
    map->AddPoint(0.912552, 8.39149);
    map->AddPoint(0.928444, 9.33991);
    map->AddPoint(0.93938, 10.144);
    map->AddPoint(1, 12.1852);
    
}

void Duro_Powertrain::SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) {
    map->AddPoint(0.0, 1.96);
    map->AddPoint(0.85, 1.0);
    map->AddPoint(1.00, 1.00);
}

}  // namespace duro
}  // end namespace vehicle
}  // end namespace chrono
