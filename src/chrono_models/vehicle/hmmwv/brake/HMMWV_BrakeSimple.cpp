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
// HMMWV simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/hmmwv/brake/HMMWV_BrakeSimple.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double HMMWV_BrakeSimple::m_maxtorque = 4000;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_BrakeSimple::HMMWV_BrakeSimple(const std::string& name) : ChBrakeSimple(name) {}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
