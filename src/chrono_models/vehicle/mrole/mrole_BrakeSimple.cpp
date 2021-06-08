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
// mrole simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/mrole/mrole_BrakeSimple.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double mrole_BrakeSimple::m_maxtorque = 10000;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
mrole_BrakeSimple::mrole_BrakeSimple(const std::string& name) : ChBrakeSimple(name) {}

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
