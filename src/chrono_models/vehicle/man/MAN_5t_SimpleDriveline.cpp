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
// MAN 5t simple driveline model.
//
// =============================================================================

#include "chrono_models/vehicle/man/MAN_5t_SimpleDriveline.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double MAN_5t_SimpleDriveline::m_front_torque_frac = 0.5;
const double MAN_5t_SimpleDriveline::m_front_diff_bias = 2.0;
const double MAN_5t_SimpleDriveline::m_rear_diff_bias = 2.0;
const double MAN_5t_SimpleDriveline::m_front_conicalgear_ratio = 0.25;
const double MAN_5t_SimpleDriveline::m_rear_conicalgear_ratio = 0.25;

// -----------------------------------------------------------------------------

MAN_5t_SimpleDriveline::MAN_5t_SimpleDriveline(const std::string& name) : ChSimpleDriveline(name) {}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
