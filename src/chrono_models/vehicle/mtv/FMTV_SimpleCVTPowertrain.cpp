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
// Simple powertrain model for the FMTV vehicles.
// - hyperbolic speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "chrono_models/vehicle/mtv/FMTV_SimpleCVTPowertrain.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

const double FMTV_SimpleCVTPowertrain::m_max_torque = 772;
const double FMTV_SimpleCVTPowertrain::m_max_power = 171000;
const double FMTV_SimpleCVTPowertrain::m_fwd_gear_ratio = 1.0;
const double FMTV_SimpleCVTPowertrain::m_rev_gear_ratio = -1.0;
const double FMTV_SimpleCVTPowertrain::m_critical_speed =
    FMTV_SimpleCVTPowertrain::m_max_power / FMTV_SimpleCVTPowertrain::m_max_torque;

FMTV_SimpleCVTPowertrain::FMTV_SimpleCVTPowertrain(const std::string& name) : ChSimpleCVTPowertrain(name) {}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
