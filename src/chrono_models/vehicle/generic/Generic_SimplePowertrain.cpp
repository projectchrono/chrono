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
// Simple powertrain model for the Generic vehicle.
// - trivial speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "chrono_models/vehicle/generic/Generic_SimplePowertrain.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Generic_SimplePowertrain::m_max_torque = 365.0;
const double Generic_SimplePowertrain::m_max_speed = 5000;
const double Generic_SimplePowertrain::m_fwd_gear_ratio = 0.3;
const double Generic_SimplePowertrain::m_rev_gear_ratio = -0.3;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Generic_SimplePowertrain::Generic_SimplePowertrain() : ChSimplePowertrain() {}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
