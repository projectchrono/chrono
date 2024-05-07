// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// Brake classe for Jeep Cherokee 1997
// Vehicle Parameters taken from SAE Paper 1999-01-0121
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "Cherokee_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace jeep {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Cherokee_BrakeShafts::m_maxtorque = 4000;
const double Cherokee_BrakeShafts::m_shaft_inertia = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Cherokee_BrakeShafts::Cherokee_BrakeShafts(const std::string& name) : ChBrakeShafts(name) {}

}  // end namespace jeep
}  // end namespace vehicle
}  // end namespace chrono
