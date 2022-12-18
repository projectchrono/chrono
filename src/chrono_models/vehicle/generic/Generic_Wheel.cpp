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
// HMMWV wheel subsystem
//
// =============================================================================

#include "chrono_models/vehicle/generic/Generic_Wheel.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Generic_Wheel::m_mass = 18.0;
const ChVector<> Generic_Wheel::m_inertia(0.3, 0.5, 0.3);

const double Generic_Wheel::m_radius = 0.3099;
const double Generic_Wheel::m_width = 0.235;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Generic_Wheel::Generic_Wheel(const std::string& name) : ChWheel(name) {}


}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
