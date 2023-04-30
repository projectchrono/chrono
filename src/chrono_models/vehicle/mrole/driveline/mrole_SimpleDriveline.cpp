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
// mrole simple driveline model.
//
// =============================================================================

#include "chrono_models/vehicle/mrole/driveline/mrole_SimpleDriveline.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double mrole_SimpleDriveline::m_front_torque_frac = 0.5;
const double mrole_SimpleDriveline::m_front_diff_bias = 2.0;
const double mrole_SimpleDriveline::m_rear_diff_bias = 2.0;

// -----------------------------------------------------------------------------
// Constructor of mrole_SimpleDriveline.
// -----------------------------------------------------------------------------
mrole_SimpleDriveline::mrole_SimpleDriveline(const std::string& name) : ChSimpleDriveline(name) {}

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
