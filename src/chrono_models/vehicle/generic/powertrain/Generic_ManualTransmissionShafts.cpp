// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Marcel Offermans
// =============================================================================
//
// Generic vehicle manual transmission model based on ChShaft objects.
//
// =============================================================================

#include "Generic_ManualTransmissionShafts.h"

namespace chrono {
namespace vehicle {
namespace generic {

// Static variables
const double Generic_ManualTransmissionShafts::m_transmissionblock_inertia = 10.5;
const double Generic_ManualTransmissionShafts::m_motorshaft_inertia = 0.5;
const double Generic_ManualTransmissionShafts::m_driveshaft_inertia = 0.5;
const double Generic_ManualTransmissionShafts::m_ingear_shaft_inertia = 0.3;
const double Generic_ManualTransmissionShafts::m_clutch_torque_limit = 1000;

Generic_ManualTransmissionShafts::Generic_ManualTransmissionShafts(const std::string& name)
    : ChManualTransmissionShafts(name) {}

void Generic_ManualTransmissionShafts::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.1;  // reverse gear;

    fwd.push_back(0.2);  // 1st gear;
    fwd.push_back(0.4);  // 2nd gear;
    fwd.push_back(0.8);  // 3rd gear;
}

}  // namespace generic
}  // namespace vehicle
}  // namespace chrono