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
// U401 shafts-based brake model.
//
// =============================================================================

#include "chrono_models/vehicle/unimog/U401_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace unimog {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double U401_BrakeShafts::m_maxtorque = 4000;
const double U401_BrakeShafts::m_shaft_inertia = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
U401_BrakeShafts::U401_BrakeShafts(const std::string& name) : ChBrakeShafts(name) {}

}  // namespace unimog
}  // end namespace vehicle
}  // end namespace chrono
