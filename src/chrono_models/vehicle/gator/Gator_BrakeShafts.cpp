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
// Gator shafts-based brake model.
//
// =============================================================================

#include "chrono_models/vehicle/gator/Gator_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace gator {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Gator_BrakeShafts::m_maxtorque = 800;
const double Gator_BrakeShafts::m_shaft_inertia = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Gator_BrakeShafts::Gator_BrakeShafts(const std::string& name) : ChBrakeShafts(name) {}

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono
