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
// FMTV shafts-based brake model.
//
// =============================================================================

#include "chrono_models/vehicle/mtv/FMTV_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double FMTV_BrakeShafts::m_maxtorque = 7500;
const double FMTV_BrakeShafts::m_shaft_inertia = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FMTV_BrakeShafts::FMTV_BrakeShafts(const std::string& name) : ChBrakeShafts(name) {}

}  // end namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
