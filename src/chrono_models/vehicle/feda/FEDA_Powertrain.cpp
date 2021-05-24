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
// FEDA powertrain model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/feda/FEDA_Powertrain.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double FEDA_Powertrain::m_motorblock_inertia = 10.5;
const double FEDA_Powertrain::m_crankshaft_inertia = 1.1;
const double FEDA_Powertrain::m_ingear_shaft_inertia = 0.3;

const double FEDA_Powertrain::m_upshift_RPM = 2300;
const double FEDA_Powertrain::m_downshift_RPM = 1200;

// -----------------------------------------------------------------------------
// Constructor of the HMMW_Powertrain.
// the direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
FEDA_Powertrain::FEDA_Powertrain(const std::string& name) : ChShaftsPowertrain(name, ChVector<>(1, 0, 0)) {
    SetGearShiftLatency(1.0);
}

// -----------------------------------------------------------------------------
// Initialize vector of gear ratios
// -----------------------------------------------------------------------------
void FEDA_Powertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -1.0 / 3.81;  // reverse gear;

    fwd.push_back(1.0 / 3.74);   // 1st gear;
    fwd.push_back(1.0 / 2.003);  // 2nd gear;
    fwd.push_back(1.0 / 1.343);  // 3rd gear;
    fwd.push_back(1.0 / 1.0);    // 4th gear;
    fwd.push_back(1.0 / 0.773);  // 5th gear;
    fwd.push_back(1.0 / 0.634);  // 5th gear;
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
void FEDA_Powertrain::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-100 * rpm_to_radsec, 300.0);  // to start engine
    map->AddPoint(700 * rpm_to_radsec, 400);
    map->AddPoint(800 * rpm_to_radsec, 410);
    map->AddPoint(900 * rpm_to_radsec, 450);
    map->AddPoint(1000 * rpm_to_radsec, 550);
    map->AddPoint(1100 * rpm_to_radsec, 625);
    map->AddPoint(1200 * rpm_to_radsec, 700);
    map->AddPoint(1300 * rpm_to_radsec, 700);
    map->AddPoint(1400 * rpm_to_radsec, 700);
    map->AddPoint(1500 * rpm_to_radsec, 700);
    map->AddPoint(1600 * rpm_to_radsec, 700);
    map->AddPoint(1700 * rpm_to_radsec, 698);
    map->AddPoint(1800 * rpm_to_radsec, 695);
    map->AddPoint(1900 * rpm_to_radsec, 669);
    map->AddPoint(2000 * rpm_to_radsec, 639);
    map->AddPoint(2100 * rpm_to_radsec, 612);
    map->AddPoint(2200 * rpm_to_radsec, 586);
    map->AddPoint(2300 * rpm_to_radsec, 562);
    map->AddPoint(2400 * rpm_to_radsec, 540);
    map->AddPoint(2500 * rpm_to_radsec, 520);
    map->AddPoint(2525 * rpm_to_radsec, 505);
    map->AddPoint(2850 * rpm_to_radsec, -400);  // fading out of engine torque
}

void FEDA_Powertrain::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-50 * rpm_to_radsec, 40.0);  // it should never work in negative direction, anyway..
    map->AddPoint(100 * rpm_to_radsec, 0.0);

    map->AddPoint(700 * rpm_to_radsec, -40.0);
    map->AddPoint(800 * rpm_to_radsec, -41.0);
    map->AddPoint(900 * rpm_to_radsec, -43.0);
    map->AddPoint(1000 * rpm_to_radsec, -44.0);
    map->AddPoint(1100 * rpm_to_radsec, -46.1);
    map->AddPoint(1200 * rpm_to_radsec, -48.7);
    map->AddPoint(1300 * rpm_to_radsec, -51.1);
    map->AddPoint(1400 * rpm_to_radsec, -55.2);
    map->AddPoint(1500 * rpm_to_radsec, -60.5);
    map->AddPoint(1600 * rpm_to_radsec, -64.5);
    map->AddPoint(1700 * rpm_to_radsec, -68.7);
    map->AddPoint(1800 * rpm_to_radsec, -71.0);
    map->AddPoint(1900 * rpm_to_radsec, -73.4);
    map->AddPoint(2000 * rpm_to_radsec, -76.8);
    map->AddPoint(2100 * rpm_to_radsec, -80.8);
    map->AddPoint(2200 * rpm_to_radsec, -85.5);
    map->AddPoint(2300 * rpm_to_radsec, -89.7);
    map->AddPoint(2400 * rpm_to_radsec, -94.6);
    map->AddPoint(2500 * rpm_to_radsec, -95.8);
    map->AddPoint(2525 * rpm_to_radsec, -95.9);
    map->AddPoint(2850 * rpm_to_radsec, -99.9);
}

void FEDA_Powertrain::SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) {
    map->AddPoint(0.0, 15);
    map->AddPoint(0.25, 15);
    map->AddPoint(0.50, 15);
    map->AddPoint(0.75, 16);
    map->AddPoint(0.90, 18);
    map->AddPoint(1.00, 35);
    /*
       map->AddPoint(0     ,   81.0000);
       map->AddPoint(0.1000,   81.1589);
       map->AddPoint(0.2000,   81.3667);
       map->AddPoint(0.3000,   81.6476);
       map->AddPoint(0.4000,   82.0445);
       map->AddPoint(0.5000,   82.6390);
       map->AddPoint(0.6000,   83.6067);
       map->AddPoint(0.7000,   85.3955);
       map->AddPoint(0.8000,   89.5183);
       map->AddPoint(0.9000,  105.1189);
       map->AddPoint(0.9700,  215.5284);
       map->AddPoint(1.0000,  235.5284);
   */
}

void FEDA_Powertrain::SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) {
    map->AddPoint(0.0, 2.00);
    map->AddPoint(0.25, 1.80);
    map->AddPoint(0.50, 1.50);
    map->AddPoint(0.75, 1.15);
    map->AddPoint(1.00, 1.00);

    /*
        map->AddPoint(0,        1.7500);
        map->AddPoint(0.1000,    1.6667);
        map->AddPoint(0.2000,    1.5833);
        map->AddPoint(0.3000,    1.5000);
        map->AddPoint(0.4000,    1.4167);
        map->AddPoint(0.5000,    1.3334);
        map->AddPoint(0.6000,    1.2500);
        map->AddPoint(0.7000,    1.1667);
        map->AddPoint(0.8000,    1.0834);
        map->AddPoint(0.9000,    1.0000);
        map->AddPoint(0.9700,    1.0000);
        map->AddPoint(1.0000,    1.0000);
    */
}

}  // end namespace feda
}  // end namespace vehicle
}  // end namespace chrono
