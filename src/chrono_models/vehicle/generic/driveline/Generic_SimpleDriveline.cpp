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
// Authors: Radu Serban, Mike Taylor
// =============================================================================
//
// Simple driveline model for the generic vehicle
//
// =============================================================================

#include <cmath>

#include "chrono_models/vehicle/generic/driveline/Generic_SimpleDriveline.h"

namespace chrono {
namespace vehicle {
namespace generic {

const double Generic_SimpleDriveline::m_front_torque_frac = 0.5;
const double Generic_SimpleDriveline::m_front_diff_bias = 2.0;
const double Generic_SimpleDriveline::m_rear_diff_bias = 2.0;

Generic_SimpleDriveline::Generic_SimpleDriveline(const std::string& name) : ChSimpleDriveline(name) {}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
