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
// M113 automatic transmission model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/m113/powertrain/M113_AutomaticTransmissionShafts.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// Static variables
const double M113_AutomaticTransmissionShafts::m_transmissionblock_inertia = 10.5;
const double M113_AutomaticTransmissionShafts::m_motorshaft_inertia = 0.5;
const double M113_AutomaticTransmissionShafts::m_driveshaft_inertia = 0.5;
const double M113_AutomaticTransmissionShafts::m_ingear_shaft_inertia = 0.3;
const double M113_AutomaticTransmissionShafts::m_upshift_RPM = 1500;
const double M113_AutomaticTransmissionShafts::m_downshift_RPM = 1000;

M113_AutomaticTransmissionShafts::M113_AutomaticTransmissionShafts(const std::string& name)
    : ChAutomaticTransmissionShafts(name) {
    SetGearShiftLatency(1.0);
}

void M113_AutomaticTransmissionShafts::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.151;  // reverse gear;

    fwd.push_back(0.240);   // 1st gear;
    fwd.push_back(0.427);   // 2nd gear;
    fwd.push_back(0.685);   // 3rd gear;
    fwd.push_back(0.962);   // 4th gear;
}

void M113_AutomaticTransmissionShafts::SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) {
    map->AddPoint(0.00, 7);
    map->AddPoint(0.25, 7);
    map->AddPoint(0.50, 7);
    map->AddPoint(0.75, 8);
    map->AddPoint(0.90, 9);
    map->AddPoint(1.00, 18);
}

void M113_AutomaticTransmissionShafts::SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) {
    map->AddPoint(0.00, 2.00);
    map->AddPoint(0.25, 1.80);
    map->AddPoint(0.50, 1.50);
    map->AddPoint(0.75, 1.15);
    map->AddPoint(0.90, 1.00);
    map->AddPoint(1.00, 1.00);
}

}  // end namespace M113
}  // end namespace vehicle
}  // end namespace chrono

