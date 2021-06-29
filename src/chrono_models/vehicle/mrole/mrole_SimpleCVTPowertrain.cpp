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
// Simple powertrain model for the mrole vehicle.
// - hyperbolic speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "chrono_models/vehicle/mrole/mrole_SimpleCVTPowertrain.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double mrole_SimpleCVTPowertrain::m_max_torque = 219363.984;
const double mrole_SimpleCVTPowertrain::m_max_power = 533000;
const double mrole_SimpleCVTPowertrain::m_max_speed = 25;
const double mrole_SimpleCVTPowertrain::m_fwd_gear_ratio = 1.0;
const double mrole_SimpleCVTPowertrain::m_rev_gear_ratio = -1.0;

// -----------------------------------------------------------------------------
mrole_SimpleCVTPowertrain::mrole_SimpleCVTPowertrain(const std::string& name) : ChSimpleCVTPowertrain(name) {}

void mrole_SimpleCVTPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear_ratio;
    fwd.push_back(m_fwd_gear_ratio);
}

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
