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
// Sedan shafts-based brake model.
//
// =============================================================================

#include "chrono_models/vehicle/sedan/Sedan_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace sedan {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Sedan_BrakeShafts::m_maxtorque = 2000;
const double Sedan_BrakeShafts::m_shaft_inertia = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Sedan_BrakeShafts::Sedan_BrakeShafts(const std::string& name) : ChBrakeShafts(name) {}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
