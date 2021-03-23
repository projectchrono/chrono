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
// Simple powertrain model for the M113 vehicle.
// - simple speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "chrono_models/vehicle/marder/Marder_SimplePowertrain.h"

namespace chrono {
namespace vehicle {
namespace marder {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Marder_SimplePowertrain::m_max_torque = 1000;
const double Marder_SimplePowertrain::m_max_speed = 18850;  // 3000 RPM
const double Marder_SimplePowertrain::m_fwd_gear_ratio = 0.05;
const double Marder_SimplePowertrain::m_rev_gear_ratio = -0.3;

// -----------------------------------------------------------------------------
Marder_SimplePowertrain::Marder_SimplePowertrain(const std::string& name) : ChSimplePowertrain(name) {}

void Marder_SimplePowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear_ratio;
    fwd.push_back(m_fwd_gear_ratio);
}

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono
