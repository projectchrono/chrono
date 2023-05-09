// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// Duro shafts-based brake model.
//
// =============================================================================

#include "chrono_models/vehicle/duro/Duro_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace duro {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Duro_BrakeShafts::m_maxtorque = 4000;
const double Duro_BrakeShafts::m_shaft_inertia = 0.5;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Duro_BrakeShafts::Duro_BrakeShafts(const std::string& name) : ChBrakeShafts(name) {}

}  // namespace duro
}  // end namespace vehicle
}  // end namespace chrono
