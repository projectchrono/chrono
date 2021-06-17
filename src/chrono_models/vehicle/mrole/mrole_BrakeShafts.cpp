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
// mrole shafts-based brake model.
//
// =============================================================================

#include "chrono_models/vehicle/mrole/mrole_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double mrole_BrakeShafts::m_maxtorque = 10000;
const double mrole_BrakeShafts::m_shaft_inertia = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
mrole_BrakeShafts::mrole_BrakeShafts(const std::string& name) : ChBrakeShafts(name) {}

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
