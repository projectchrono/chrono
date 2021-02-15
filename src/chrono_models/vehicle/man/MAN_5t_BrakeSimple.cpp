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
// MAN 5t simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/man/MAN_5t_BrakeSimple.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double MAN_5t_BrakeSimple::m_maxtorque = 10000;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MAN_5t_BrakeSimple::MAN_5t_BrakeSimple(const std::string& name) : ChBrakeSimple(name) {}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
