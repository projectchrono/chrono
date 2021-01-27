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
// FED-alpha shafts-based brake model.
//
// =============================================================================

#include "chrono_models/vehicle/feda/FEDA_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double FEDA_BrakeShafts::m_maxtorque = 4000;
const double FEDA_BrakeShafts::m_shaft_inertia = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FEDA_BrakeShafts::FEDA_BrakeShafts(const std::string& name) : ChBrakeShafts(name) {}

}  // end namespace feda
}  // end namespace vehicle
}  // end namespace chrono
