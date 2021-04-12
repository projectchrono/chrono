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
// Authors:
// =============================================================================
//
// M113 powertrain model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/m113/M113_ShaftsPowertrain.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_ShaftsPowertrain::m_motorblock_inertia = 10.5;
const double M113_ShaftsPowertrain::m_crankshaft_inertia = 1.1;
const double M113_ShaftsPowertrain::m_ingear_shaft_inertia = 0.3;

const double M113_ShaftsPowertrain::m_upshift_RPM = 1500;
const double M113_ShaftsPowertrain::m_downshift_RPM = 1000;

// -----------------------------------------------------------------------------
// Constructor of the M113_ShaftsPowertrain.
// the direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
M113_ShaftsPowertrain::M113_ShaftsPowertrain(const std::string& name) : ChShaftsPowertrain(name, ChVector<>(1, 0, 0)) {
    SetGearShiftLatency(0.5);
}

// -----------------------------------------------------------------------------
// Initialize vector of gear ratios
// https://www.nsncenter.com/Files/library/TM/M113/TM-9-2520-272-34P/TM-9-2520-272-34P.pdf
// -----------------------------------------------------------------------------
void M113_ShaftsPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.151;  // reverse gear;

    fwd.push_back(0.240);   // 1st gear;
    fwd.push_back(0.427);   // 2nd gear;
    fwd.push_back(0.685);   // 3rd gear;
    fwd.push_back(0.962);   // 4th gear;
}

// -----------------------------------------------------------------------------
// Set the engine and torque converter maps:
//
// (1) engine speed [rad/s] - torque [Nm] map
//     must be defined beyond max speed too - engine might be 'pulled'
//     http://powerforce.com/PDFs/2Cycle_Engines/DS_PF6V-53N.pdf
//
// (2) TC capacity factor map
//
// (3) TC torque ratio map
//
// -----------------------------------------------------------------------------
void M113_ShaftsPowertrain::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
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

void M113_ShaftsPowertrain::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-50 * rpm_to_radsec, 30);  // it should never work in negative direction, anyway..
    map->AddPoint(0 * rpm_to_radsec, 0);
    map->AddPoint(50 * rpm_to_radsec, -30);
    map->AddPoint(1000 * rpm_to_radsec, -50);
    map->AddPoint(2000 * rpm_to_radsec, -70);
    map->AddPoint(3000 * rpm_to_radsec, -90);
}

void M113_ShaftsPowertrain::SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) {
    map->AddPoint(0.00, 7);
    map->AddPoint(0.25, 7);
    map->AddPoint(0.50, 7);
    map->AddPoint(0.75, 8);
    map->AddPoint(0.90, 9);
    map->AddPoint(1.00, 18);

    ////map->AddPoint(0.0000, 12.2938);
    ////map->AddPoint(0.5000, 12.8588);
    ////map->AddPoint(0.6000, 13.1452);
    ////map->AddPoint(0.7000, 13.6285);
    ////map->AddPoint(0.8000, 14.6163);
    ////map->AddPoint(0.8700, 16.2675);
    ////map->AddPoint(0.9200, 19.3503);
    ////map->AddPoint(0.9400, 22.1046);
    ////map->AddPoint(0.9600, 29.9986);
    ////map->AddPoint(0.9700, 50.0000);
}

void M113_ShaftsPowertrain::SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) {
    map->AddPoint(0.00, 2.00);
    map->AddPoint(0.25, 1.80);
    map->AddPoint(0.50, 1.50);
    map->AddPoint(0.75, 1.15);
    map->AddPoint(0.90, 1.00);
    map->AddPoint(1.00, 1.00);

    ////map->AddPoint(0.0000, 2.2320);
    ////map->AddPoint(0.5000, 1.5462);
    ////map->AddPoint(0.6000, 1.4058);
    ////map->AddPoint(0.7000, 1.2746);
    ////map->AddPoint(0.8000, 1.1528);
    ////map->AddPoint(0.8700, 1.0732);
    ////map->AddPoint(0.9200, 1.0192);
    ////map->AddPoint(0.9400, 0.9983);
    ////map->AddPoint(0.9600, 0.9983);
    ////map->AddPoint(0.9700, 0.9983);
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
