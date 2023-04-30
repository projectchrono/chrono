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
// Generic transmission model for a vehicle with no torque converter and a
// single forward gear.
//
// =============================================================================

#include "subsystems/ACV_AutomaticTransmissionSimple.h"

using namespace chrono;
using namespace chrono::vehicle;

const double ACV_AutomaticTransmissionSimple::m_fwd_gear_ratio = 0.3;
const double ACV_AutomaticTransmissionSimple::m_rev_gear_ratio = -0.3;

const double rpm2rads = CH_C_PI / 30;

ACV_AutomaticTransmissionSimple::ACV_AutomaticTransmissionSimple(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void ACV_AutomaticTransmissionSimple::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear_ratio;
    fwd.push_back(m_fwd_gear_ratio);
}

void ACV_AutomaticTransmissionSimple::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    shift_bands.push_back(std::pair<double, double>(750 * rpm2rads, 1500 * rpm2rads));
}
