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
// mrole powertrain model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/mrole/mrole_Powertrain.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double mrole_Powertrain::m_motorblock_inertia = 10.5;
const double mrole_Powertrain::m_crankshaft_inertia = 1.1;
const double mrole_Powertrain::m_ingear_shaft_inertia = 0.3;

const double mrole_Powertrain::m_upshift_RPM = 2000;
const double mrole_Powertrain::m_downshift_RPM = 1200;

// -----------------------------------------------------------------------------
// Constructor of the HMMW_Powertrain.
// the direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
mrole_Powertrain::mrole_Powertrain(const std::string& name) : ChShaftsPowertrain(name, ChVector<>(1, 0, 0)) {
    SetGearShiftLatency(1.5);
}

// -----------------------------------------------------------------------------
// Initialize vector of gear ratios
// -----------------------------------------------------------------------------
void mrole_Powertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    // Allison 4500
    rev = -0.12;  // reverse gear;

    fwd.push_back(0.14);  // 1st gear;
    fwd.push_back(0.30);  // 2nd gear;
    fwd.push_back(0.44);  // 3rd gear;
    fwd.push_back(0.67);  // 4rd gear;
    fwd.push_back(0.88);  // 5rd gear;
    fwd.push_back(1.00);  // 6rd gear;
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
void mrole_Powertrain::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    /* map->AddPoint(-100 * rpm_to_radsec, 300);  // to start engine
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
    map->AddPoint(2700 * rpm_to_radsec, -400);
    fading out of engine torque */

    map->AddPoint(-100 * rpm_to_radsec, 500);
    map->AddPoint(702.26 * rpm_to_radsec, 600);
    map->AddPoint(1083.8 * rpm_to_radsec, 822.2369047);
    map->AddPoint(1207.97 * rpm_to_radsec, 1337.962406);
    map->AddPoint(1275.7 * rpm_to_radsec, 1649.587276);
    map->AddPoint(1370.52 * rpm_to_radsec, 1997.484418);
    map->AddPoint(1460.82 * rpm_to_radsec, 2171.572354);
    map->AddPoint(1553.39 * rpm_to_radsec, 2253.81817);
    map->AddPoint(1645.95 * rpm_to_radsec, 2318.762639);
    map->AddPoint(1738.51 * rpm_to_radsec, 2353.941623);
    map->AddPoint(1828.82 * rpm_to_radsec, 2345.10782);
    map->AddPoint(1930.41 * rpm_to_radsec, 2323.30056);
    map->AddPoint(2016.2 * rpm_to_radsec, 2289.140971);
    map->AddPoint(2111.02 * rpm_to_radsec, 2204.05267);
    map->AddPoint(2210.36 * rpm_to_radsec, 2121.9315);
    map->AddPoint(2298.41 * rpm_to_radsec, 2051.236819);
    map->AddPoint(2450 * rpm_to_radsec, -1000.0);
}

void mrole_Powertrain::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(-50 * rpm_to_radsec, 30);  // it should never work in negative direction, anyway..
    map->AddPoint(0 * rpm_to_radsec, 0);
    map->AddPoint(50 * rpm_to_radsec, -300);
    map->AddPoint(1000 * rpm_to_radsec, -500);
    map->AddPoint(2000 * rpm_to_radsec, -700);
    map->AddPoint(3000 * rpm_to_radsec, -900);
}

// Torque Converter Allison TC-521
void mrole_Powertrain::SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) {
    /*
    map->AddPoint(0.0, 15);
    map->AddPoint(0.25, 15);
    map->AddPoint(0.50, 15);
    map->AddPoint(0.75, 16);
    map->AddPoint(0.90, 18);
    map->AddPoint(1.00, 35);
    */
    map->AddPoint(0.000000, 3.224532);
    map->AddPoint(0.100000, 3.231949);
    map->AddPoint(0.200000, 3.246434);
    map->AddPoint(0.300000, 3.270330);
    map->AddPoint(0.400000, 3.306386);
    map->AddPoint(0.500000, 3.358039);
    map->AddPoint(0.600000, 3.432812);
    map->AddPoint(0.700000, 3.541184);
    map->AddPoint(0.800000, 3.703053);
    map->AddPoint(0.850000, 3.788778);
    map->AddPoint(0.900000, 4.297625);
    map->AddPoint(0.950000, 5.244312);
    map->AddPoint(1.000000, 13.488155);

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

void mrole_Powertrain::SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) {
    map->AddPoint(0.0, 2.43);
    map->AddPoint(0.85, 1.00);
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

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
