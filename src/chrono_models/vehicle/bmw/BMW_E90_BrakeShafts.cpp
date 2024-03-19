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
// BMW E90 shafts-based brake model.
// Patrameters todo
//
// =============================================================================

#include "chrono_models/vehicle/bmw/BMW_E90_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace bmw {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double BMW_E90_BrakeShafts::m_maxtorque = 2000;
const double BMW_E90_BrakeShafts::m_shaft_inertia = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
BMW_E90_BrakeShafts::BMW_E90_BrakeShafts(const std::string& name) : ChBrakeShafts(name) {}

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono
