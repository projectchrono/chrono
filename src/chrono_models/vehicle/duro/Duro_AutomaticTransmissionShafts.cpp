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
// Duro automatic transmission model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/duro/Duro_AutomaticTransmissionShafts.h"

namespace chrono {
namespace vehicle {
namespace duro {

// Static variables
const double Duro_AutomaticTransmissionShafts::m_transmissionblock_inertia = 10.5;
const double Duro_AutomaticTransmissionShafts::m_motorshaft_inertia = 0.5;
const double Duro_AutomaticTransmissionShafts::m_driveshaft_inertia = 0.5;
const double Duro_AutomaticTransmissionShafts::m_ingear_shaft_inertia = 0.3;
const double Duro_AutomaticTransmissionShafts::m_upshift_RPM = 2500;
const double Duro_AutomaticTransmissionShafts::m_downshift_RPM = 1200;

Duro_AutomaticTransmissionShafts::Duro_AutomaticTransmissionShafts(const std::string& name)
    : ChAutomaticTransmissionShafts(name) {
    SetGearShiftLatency(1.0);
}

void Duro_AutomaticTransmissionShafts::SetGearRatios(std::vector<double>& fwd, double& rev) {
    // automatic gearbox Mercedes Benz W 4 A 028
    rev = -1.0 / 5.586;  // reverse gear;

    fwd.push_back(1.0 / 3.871);  // 1st gear;
    fwd.push_back(1.0 / 2.247);  // 2nd gear;
    fwd.push_back(1.0 / 1.436);  // 3rd gear;
    fwd.push_back(1.0);          // 4th gear;
}

void Duro_AutomaticTransmissionShafts::SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) {
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

void Duro_AutomaticTransmissionShafts::SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) {
    map->AddPoint(0.0, 1.96);
    map->AddPoint(0.85, 1.0);
    map->AddPoint(1.00, 1.00);
}

}  // end namespace Duro
}  // end namespace vehicle
}  // end namespace chrono

