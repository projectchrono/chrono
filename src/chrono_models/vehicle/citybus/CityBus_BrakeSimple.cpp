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
// Authors: Asher Elmquist, Evan Hoerl
// =============================================================================
//
// CityBus simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/citybus/CityBus_BrakeSimple.h"

namespace chrono {
namespace vehicle {
namespace citybus {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double CityBus_BrakeSimple::m_maxtorque = 10000;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
CityBus_BrakeSimple::CityBus_BrakeSimple(const std::string& name) : ChBrakeSimple(name) {}

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
