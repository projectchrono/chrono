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
// Authors: Radu Serban
// =============================================================================
//
// Simple CVT powertrain model for the M113 vehicle.
// - simple speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "chrono_models/vehicle/m113/M113_SimpleCVTPowertrain.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_SimpleCVTPowertrain::m_max_torque = 450 / 0.73756;  // 450 lb-ft
const double M113_SimpleCVTPowertrain::m_max_power = 156597;          // 210 BHP
const double M113_SimpleCVTPowertrain::m_max_speed = 550;             // > 5000 RPM
const double M113_SimpleCVTPowertrain::m_fwd_gear_ratio = 0.240;
const double M113_SimpleCVTPowertrain::m_rev_gear_ratio = -0.151;

// -----------------------------------------------------------------------------
M113_SimpleCVTPowertrain::M113_SimpleCVTPowertrain(const std::string& name) : ChSimpleCVTPowertrain(name) {}

void M113_SimpleCVTPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear_ratio;
    fwd.push_back(m_fwd_gear_ratio);
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
