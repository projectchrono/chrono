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
// Marder automatic transmission model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/marder/powertrain/Marder_AutomaticTransmissionShafts.h"

namespace chrono {
namespace vehicle {
namespace marder {

// Static variables
const double Marder_AutomaticTransmissionShafts::m_transmissionblock_inertia = 10.5;
const double Marder_AutomaticTransmissionShafts::m_motorshaft_inertia = 0.5;
const double Marder_AutomaticTransmissionShafts::m_driveshaft_inertia = 0.5;
const double Marder_AutomaticTransmissionShafts::m_ingear_shaft_inertia = 0.6;
const double Marder_AutomaticTransmissionShafts::m_upshift_RPM = 2000;
const double Marder_AutomaticTransmissionShafts::m_downshift_RPM = 1200;

Marder_AutomaticTransmissionShafts::Marder_AutomaticTransmissionShafts(const std::string& name)
    : ChAutomaticTransmissionShafts(name) {
    SetGearShiftLatency(1.0);
}

void Marder_AutomaticTransmissionShafts::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.25;  // reverse gear;

    fwd.push_back(0.27);  // 1st gear;
    fwd.push_back(0.45);  // 2nd gear;
    fwd.push_back(0.63);  // 3rd gear;
    fwd.push_back(1.0);   // 4th gear;
}

void Marder_AutomaticTransmissionShafts::SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) {
    // Torque Converter Allison TC-521
    // calculated from SAE TC example curve
    map->AddPoint(0, 3.023);
    map->AddPoint(0.105301, 3.01349);
    map->AddPoint(0.20916, 3.023);
    map->AddPoint(0.320936, 3.06102);
    map->AddPoint(0.435681, 3.10856);
    map->AddPoint(0.556364, 3.1751);
    map->AddPoint(0.647369, 3.21312);
    map->AddPoint(0.724529, 3.27016);
    map->AddPoint(0.786849, 3.30819);
    map->AddPoint(0.843236, 3.35572);
    map->AddPoint(0.891714, 3.43177);
    map->AddPoint(0.912552, 3.86906);
    map->AddPoint(0.928444, 4.30635);
    map->AddPoint(0.93938, 4.67709);
    map->AddPoint(1, 5.61822);
}

void Marder_AutomaticTransmissionShafts::SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) {
    // calculated from SAE TC example curve
    map->AddPoint(0.0, 2.5);
    map->AddPoint(0.85, 1.0);
    map->AddPoint(1.0, 1.0);
}

}  // end namespace Marder
}  // end namespace vehicle
}  // end namespace chrono

