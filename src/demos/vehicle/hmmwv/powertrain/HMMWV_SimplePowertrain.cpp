// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Simple powertrain model for the HMMWV vehicle.
// - trivial speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "hmmwv/powertrain/HMMWV_SimplePowertrain.h"

using namespace chrono;

namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double HMMWV_SimplePowertrain::m_max_torque = 2400 / 8.851;
const double HMMWV_SimplePowertrain::m_max_speed  = 2000;
const double HMMWV_SimplePowertrain::m_fwd_gear_ratio = 0.3;
const double HMMWV_SimplePowertrain::m_rev_gear_ratio = -0.3;


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_SimplePowertrain::HMMWV_SimplePowertrain()
: ChSimplePowertrain()
{
}


} // end namespace hmmwv
