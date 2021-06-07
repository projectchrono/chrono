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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Simple powertrain model for the mrole vehicle.
// - trivial speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "chrono_models/vehicle/mrole/mrole_SimplePowertrain.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double mrole_SimplePowertrain::m_max_torque = 2400 / 8.851;
const double mrole_SimplePowertrain::m_max_speed = 2000;
const double mrole_SimplePowertrain::m_fwd_gear_ratio = 0.3;
const double mrole_SimplePowertrain::m_rev_gear_ratio = -0.3;

// -----------------------------------------------------------------------------
mrole_SimplePowertrain::mrole_SimplePowertrain(const std::string& name) : ChSimplePowertrain(name) {}

void mrole_SimplePowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear_ratio;
    fwd.push_back(m_fwd_gear_ratio);
}

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
