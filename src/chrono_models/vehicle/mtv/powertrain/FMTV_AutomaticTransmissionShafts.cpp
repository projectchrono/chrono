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
// FMTV automatic transmission model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/mtv/powertrain/FMTV_AutomaticTransmissionShafts.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// Static variables
const double FMTV_AutomaticTransmissionShafts::m_transmissionblock_inertia = 10.5;
const double FMTV_AutomaticTransmissionShafts::m_motorshaft_inertia = 0.5;
const double FMTV_AutomaticTransmissionShafts::m_driveshaft_inertia = 0.5;
const double FMTV_AutomaticTransmissionShafts::m_ingear_shaft_inertia = 0.3;
const double FMTV_AutomaticTransmissionShafts::m_upshift_RPM = 2400;
const double FMTV_AutomaticTransmissionShafts::m_downshift_RPM = 1200;

FMTV_AutomaticTransmissionShafts::FMTV_AutomaticTransmissionShafts(const std::string& name)
    : ChAutomaticTransmissionShafts(name) {
    SetGearShiftLatency(1.0);
}

void FMTV_AutomaticTransmissionShafts::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -1.0 / 11.8;  // reverse gear

    fwd.push_back(1.0 / 11.921875);  // 1st gear
    fwd.push_back(1.0 / 5.484375);   // 2nd gear
    fwd.push_back(1.0 / 2.984375);   // 3rd gear
    fwd.push_back(1.0 / 2.234375);   // 4th gear
    fwd.push_back(1.0 / 1.5625);     // 5th gear
    fwd.push_back(1.0 / 1.15625);    // 6th gear
    fwd.push_back(1.0);              // 7th gear
}

void FMTV_AutomaticTransmissionShafts::SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) {
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

void FMTV_AutomaticTransmissionShafts::SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) {
    map->AddPoint(0.0, 2.17);
    map->AddPoint(0.85, 1.00);
    map->AddPoint(1.00, 1.00);
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
