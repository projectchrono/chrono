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
// Jeep Cherokee 1997 automatic transmission model based on ChShaft objects.
// Vehicle Parameters taken from SAE Paper 1999-01-0121
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "Cherokee_AutomaticTransmissionStafts.h"

namespace chrono {
namespace vehicle {
namespace jeep {

// Static variables
const double Cherokee_AutomaticTransmissionShafts::m_transmissionblock_inertia = 10.5;
const double Cherokee_AutomaticTransmissionShafts::m_motorshaft_inertia = 0.5;
const double Cherokee_AutomaticTransmissionShafts::m_driveshaft_inertia = 0.5;
const double Cherokee_AutomaticTransmissionShafts::m_ingear_shaft_inertia = 0.3;
const double Cherokee_AutomaticTransmissionShafts::m_upshift_RPM = 4500;
const double Cherokee_AutomaticTransmissionShafts::m_downshift_RPM = 1500;

Cherokee_AutomaticTransmissionShafts::Cherokee_AutomaticTransmissionShafts(const std::string& name)
    : ChAutomaticTransmissionShafts(name) {
    SetGearShiftLatency(1.0);
}

void Cherokee_AutomaticTransmissionShafts::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -1.0 / 2.39;  // reverse gear;

    fwd.push_back(1.0 / 2.8);   // 1st gear;
    fwd.push_back(1.0 / 1.55);  // 2nd gear;
    fwd.push_back(1.0);         // 3rd gear;
    fwd.push_back(1.0 / 0.75);  // 4rd gear;
}

void Cherokee_AutomaticTransmissionShafts::SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunctionInterp>& map) {
    // assumpted stall speed = 2500 rpm
    map->AddPoint(0.0,17.65);
    map->AddPoint(0.1,17.58);
    map->AddPoint(0.2,17.77);
    map->AddPoint(0.3,17.97);
    map->AddPoint(0.4,18.55);
    map->AddPoint(0.5,19.45);
    map->AddPoint(0.6,20.48);
    map->AddPoint(0.7,21.90);
    map->AddPoint(0.8,23.70);
    map->AddPoint(0.86,24.73);
    map->AddPoint(0.9,29.5);
    map->AddPoint(0.92,33.11);
    map->AddPoint(0.94,39.62);
    map->AddPoint(0.98,46.26);
    map->AddPoint(0.99,52.70);
    map->AddPoint(1.00,54.95);
}

void Cherokee_AutomaticTransmissionShafts::SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunctionInterp>& map) {
    map->AddPoint(0.0, 2.05);
    map->AddPoint(0.85, 1.0);
    map->AddPoint(1.00, 1.0);
}

}  // namespace jeep
}  // end namespace vehicle
}  // end namespace chrono
