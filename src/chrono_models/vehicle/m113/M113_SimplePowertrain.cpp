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

#include "chrono_models/vehicle/m113/M113_SimplePowertrain.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_SimplePowertrain::m_max_torque = 600;
const double M113_SimplePowertrain::m_max_speed = 350;
const double M113_SimplePowertrain::m_fwd_gear_ratio = 0.240;
const double M113_SimplePowertrain::m_rev_gear_ratio = -0.151;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_SimplePowertrain::M113_SimplePowertrain(const std::string& name) : ChSimplePowertrain(name) {}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
