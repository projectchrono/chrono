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
// Simple powertrain model for the HMMWV vehicle.
// - hyperbolic speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "chrono_models/vehicle/hmmwv/HMMWV_SimpleCVTPowertrain.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double HMMWV_SimpleCVTPowertrain::m_max_torque = 330;
const double HMMWV_SimpleCVTPowertrain::m_max_power = 110000;
const double HMMWV_SimpleCVTPowertrain::m_max_speed = 10000;
const double HMMWV_SimpleCVTPowertrain::m_fwd_gear_ratio = 0.3;
const double HMMWV_SimpleCVTPowertrain::m_rev_gear_ratio = -0.3;

// -----------------------------------------------------------------------------
HMMWV_SimpleCVTPowertrain::HMMWV_SimpleCVTPowertrain(const std::string& name) : ChSimpleCVTPowertrain(name) {}

void HMMWV_SimpleCVTPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear_ratio;
    fwd.push_back(m_fwd_gear_ratio);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
