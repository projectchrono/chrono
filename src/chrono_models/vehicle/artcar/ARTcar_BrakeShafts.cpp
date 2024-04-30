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
// ARTcar shafts-based brake model.
//
// =============================================================================

#include "chrono_models/vehicle/artcar/ARTcar_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace artcar {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double ARTcar_BrakeShafts::m_maxtorque = 1;         // todo
const double ARTcar_BrakeShafts::m_shaft_inertia = 0.04;  // todo

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ARTcar_BrakeShafts::ARTcar_BrakeShafts(const std::string& name) : ChBrakeShafts(name) {}

}  // namespace artcar
}  // end namespace vehicle
}  // end namespace chrono
