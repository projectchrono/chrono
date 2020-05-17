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
// Simple powertrain model for the Gator vehicle.
// - linear speed-torque curve
// - no torque converter
// - no transmission box (single forward gear, single reverse gear)
//
// =============================================================================

#include "chrono_models/vehicle/gator/Gator_SimplePowertrain.h"

namespace chrono {
namespace vehicle {
namespace gator {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Gator_SimplePowertrain::m_max_torque = 80;
const double Gator_SimplePowertrain::m_max_speed = 3500 * (CH_C_2PI / 60.0);
const double Gator_SimplePowertrain::m_fwd_gear_ratio = 0.3;
const double Gator_SimplePowertrain::m_rev_gear_ratio = -0.5;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Gator_SimplePowertrain::Gator_SimplePowertrain(const std::string& name) : ChSimplePowertrain(name) {}

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono
