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
// Authors: Alessandro Tasora
// =============================================================================
//
// U401 simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/unimog/U401_BrakeSimple.h"

namespace chrono {
namespace vehicle {
namespace unimog {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double U401_BrakeSimple::m_maxtorque = 4000;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
U401_BrakeSimple::U401_BrakeSimple(const std::string& name) : ChBrakeSimple(name) {}

}  // namespace unimog
}  // end namespace vehicle
}  // end namespace chrono
