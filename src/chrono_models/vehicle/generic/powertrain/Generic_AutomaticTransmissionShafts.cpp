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
// Generic vehicle automatic transmission model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/generic/powertrain/Generic_AutomaticTransmissionShafts.h"

namespace chrono {
namespace vehicle {
namespace generic {

// Static variables
const double Generic_AutomaticTransmissionShafts::m_transmissionblock_inertia = 10.5;
const double Generic_AutomaticTransmissionShafts::m_motorshaft_inertia = 0.5;
const double Generic_AutomaticTransmissionShafts::m_driveshaft_inertia = 0.5;
const double Generic_AutomaticTransmissionShafts::m_ingear_shaft_inertia = 0.3;
const double Generic_AutomaticTransmissionShafts::m_upshift_RPM = 2500;
const double Generic_AutomaticTransmissionShafts::m_downshift_RPM = 1200;

Generic_AutomaticTransmissionShafts::Generic_AutomaticTransmissionShafts(const std::string& name)
    : ChAutomaticTransmissionShafts(name) {
    SetGearShiftLatency(1.0);
}

void Generic_AutomaticTransmissionShafts::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.1;  // reverse gear;

    fwd.push_back(0.2);  // 1st gear;
    fwd.push_back(0.4);  // 2nd gear;
    fwd.push_back(0.8);  // 3rd gear;
}

void Generic_AutomaticTransmissionShafts::SetTorqueConverterCapacityFactorMap(
    std::shared_ptr<ChFunction_Recorder>& map) {
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

void Generic_AutomaticTransmissionShafts::SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) {
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

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
