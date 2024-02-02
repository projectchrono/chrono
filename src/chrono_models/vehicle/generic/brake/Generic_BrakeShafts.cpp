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
// Generic vehicle shafts-based brake model.
//
// =============================================================================

#include "chrono_models/vehicle/generic/brake/Generic_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace generic {

const double Generic_BrakeShafts::m_maxtorque = 4000;
const double Generic_BrakeShafts::m_shaft_inertia = 0.4;

Generic_BrakeShafts::Generic_BrakeShafts(const std::string& name) : ChBrakeShafts(name) {}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
