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
// Authors: Rainer Gericke
// =============================================================================
//
// LMTV 2.5t simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/mtv/LMTV_BrakeSimple.h"

namespace chrono {
namespace vehicle {
namespace mtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double LMTV_BrakeSimple::m_maxtorque = 7500;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
LMTV_BrakeSimple::LMTV_BrakeSimple(const std::string& name) : ChBrakeSimple(name) {}

}  // namespace mtv
}  // end namespace vehicle
}  // end namespace chrono
