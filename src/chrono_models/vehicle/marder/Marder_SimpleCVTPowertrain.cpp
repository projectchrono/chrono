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
// Simple CVT powertrain model for the Marder vehicle.
// - simple speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "chrono_models/vehicle/marder/Marder_SimpleCVTPowertrain.h"

namespace chrono {
namespace vehicle {
namespace marder {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Marder_SimpleCVTPowertrain::m_max_torque = 10000.0;
const double Marder_SimpleCVTPowertrain::m_max_power = 800000.0;  // 600 BHP original = 444 kW
const double Marder_SimpleCVTPowertrain::m_max_speed = 550;       // > 5000 RPM
const double Marder_SimpleCVTPowertrain::m_fwd_gear_ratio = 0.240;
const double Marder_SimpleCVTPowertrain::m_rev_gear_ratio = -0.151;

// -----------------------------------------------------------------------------
Marder_SimpleCVTPowertrain::Marder_SimpleCVTPowertrain(const std::string& name) : ChSimpleCVTPowertrain(name) {}

void Marder_SimpleCVTPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear_ratio;
    fwd.push_back(m_fwd_gear_ratio);
}

}  // end namespace marder
}  // end namespace vehicle
}  // end namespace chrono
