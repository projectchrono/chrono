// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Simple powertrain model for the MAN 5t vehicle.
// - hyperbolic speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "chrono_models/vehicle/man/MAN_5t_SimpleCVTPowertrain.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double MAN_5t_SimpleCVTPowertrain::m_max_torque = 785.0 * 43.0;
const double MAN_5t_SimpleCVTPowertrain::m_max_power = 188000;
const double MAN_5t_SimpleCVTPowertrain::m_max_speed = 300;
const double MAN_5t_SimpleCVTPowertrain::m_fwd_gear_ratio = 1.0;
const double MAN_5t_SimpleCVTPowertrain::m_rev_gear_ratio = -1.0;

// -----------------------------------------------------------------------------
MAN_5t_SimpleCVTPowertrain::MAN_5t_SimpleCVTPowertrain(const std::string& name)
    : ChSimpleCVTPowertrain(name) {}

void MAN_5t_SimpleCVTPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear_ratio;
    fwd.push_back(m_fwd_gear_ratio);
}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
