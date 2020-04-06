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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// LMTV 2.5t simple driveline model.
//
// =============================================================================

#include "chrono_models/vehicle/mtv/LMTV_SimpleDriveline.h"

namespace chrono {
namespace vehicle {
namespace mtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double LMTV_SimpleDriveline::m_front_torque_frac = 0.5;
const double LMTV_SimpleDriveline::m_front_diff_bias = 2.0;
const double LMTV_SimpleDriveline::m_rear_diff_bias = 2.0;

// -----------------------------------------------------------------------------
// Constructor of LMTV_SimpleDriveline.
// -----------------------------------------------------------------------------
LMTV_SimpleDriveline::LMTV_SimpleDriveline(const std::string& name) : ChSimpleDriveline(name) {}

}  // namespace mtv
}  // end namespace vehicle
}  // end namespace chrono
