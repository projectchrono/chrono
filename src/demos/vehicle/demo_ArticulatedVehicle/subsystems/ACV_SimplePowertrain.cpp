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

#include "subsystems/ACV_SimplePowertrain.h"

using namespace chrono;
using namespace chrono::vehicle;

// Static variables
const double ACV_SimplePowertrain::m_max_torque = 300.0;
const double ACV_SimplePowertrain::m_max_speed = 4000;
const double ACV_SimplePowertrain::m_fwd_gear_ratio = 0.3;
const double ACV_SimplePowertrain::m_rev_gear_ratio = -0.3;

ACV_SimplePowertrain::ACV_SimplePowertrain(const std::string& name) : ChSimplePowertrain(name) {}
