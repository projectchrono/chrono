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
// CityBus shafts-based brake model.
//
// =============================================================================

#include "chrono_models/vehicle/citybus/CityBus_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace citybus {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double CityBus_BrakeShafts::m_maxtorque = 10000;
const double CityBus_BrakeShafts::m_shaft_inertia = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
CityBus_BrakeShafts::CityBus_BrakeShafts(const std::string& name) : ChBrakeShafts(name) {}

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
