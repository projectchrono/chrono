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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Kraz 64431 28t simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/kraz/Kraz_tractor_Brake.h"

namespace chrono {
namespace vehicle {
namespace kraz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Kraz_tractor_Brake::m_maxtorque = 10000;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Kraz_tractor_Brake::Kraz_tractor_Brake(const std::string& name) : ChBrakeSimple(name) {}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
