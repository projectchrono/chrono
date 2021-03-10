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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// FMTV powertrain model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/mtv/FMTV_Powertrain.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double FMTV_Powertrain::m_motorblock_inertia = 10.5;
const double FMTV_Powertrain::m_crankshaft_inertia = 1.1;
const double FMTV_Powertrain::m_ingear_shaft_inertia = 0.3;

const double FMTV_Powertrain::m_upshift_RPM = 2400;
const double FMTV_Powertrain::m_downshift_RPM = 1200;

// -----------------------------------------------------------------------------
// Constructor of the FMTV_Powertrain.
// the direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
FMTV_Powertrain::FMTV_Powertrain(const std::string& name) : ChShaftsPowertrain(name, ChVector<>(1, 0, 0)) {
    SetGearShiftLatency(1.0);
}

// -----------------------------------------------------------------------------
// Initialize vector of gear ratios
// -----------------------------------------------------------------------------
void FMTV_Powertrain::SetGearRatios(std::vector<double>& gear_ratios) {
    gear_ratios.push_back(-1.0 / 11.8);      // 0: reverse gear;
    gear_ratios.push_back(1.0 / 11.921875);  // 1: 1st gear;
    gear_ratios.push_back(1.0 / 5.484375);   // 2: 2nd gear;
    gear_ratios.push_back(1.0 / 2.984375);   // 3: 3rd gear;
    gear_ratios.push_back(1.0 / 2.234375);   // 4: 4th gear;
    gear_ratios.push_back(1.0 / 1.5625);     // 5: 5th gear;
    gear_ratios.push_back(1.0 / 1.15625);    // 6: 6th gear;
    gear_ratios.push_back(1.0);              // 7: 7th gear;
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
void FMTV_Powertrain::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
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

void FMTV_Powertrain::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-50 * rpm_to_radsec, 30);  // it should never work in negative direction, anyway..
    map->AddPoint(0 * rpm_to_radsec, 0);
    map->AddPoint(50 * rpm_to_radsec, -30);
    map->AddPoint(1000 * rpm_to_radsec, -50);
    map->AddPoint(2000 * rpm_to_radsec, -70);
    map->AddPoint(3000 * rpm_to_radsec, -90);
}

// TC data converted from data presented in:
// Pohl B., "Transient Torque Converter Performance, Testing, Simulation and Reverse Engineering"
// SAE Paper 2003-01-0249

void FMTV_Powertrain::SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) {
    map->AddPoint(0.000000, 6.962242);
    map->AddPoint(0.100000, 6.962242);
    map->AddPoint(0.200000, 7.053054);
    map->AddPoint(0.300000, 7.083325);
    map->AddPoint(0.400000, 7.174137);
    map->AddPoint(0.500000, 7.264949);
    map->AddPoint(0.600000, 7.355760);
    map->AddPoint(0.700000, 7.507113);
    map->AddPoint(0.800000, 7.688737);
    map->AddPoint(0.900000, 7.930902);
    map->AddPoint(1.000000, 15.135309);
}

void FMTV_Powertrain::SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) {
    map->AddPoint(0.0, 2.17);
    map->AddPoint(0.85, 1.00);
    map->AddPoint(1.00, 1.00);
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
