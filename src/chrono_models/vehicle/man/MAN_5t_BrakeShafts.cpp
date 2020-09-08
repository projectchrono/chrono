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
// MAN shafts-based brake model.
//
// =============================================================================

#include "chrono_models/vehicle/man/MAN_5t_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double MAN_5t_BrakeShafts::m_maxtorque = 10000;
const double MAN_5t_BrakeShafts::m_shaft_inertia = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MAN_5t_BrakeShafts::MAN_5t_BrakeShafts(const std::string& name) : ChBrakeShafts(name) {}

}  // end namespace man
}  // end namespace vehicle
}  // end namespace chrono
