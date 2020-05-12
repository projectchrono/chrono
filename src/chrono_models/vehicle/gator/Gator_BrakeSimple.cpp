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
// Gator simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/gator/Gator_BrakeSimple.h"

namespace chrono {
namespace vehicle {
namespace gator {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Gator_BrakeSimple::m_maxtorque = 1600;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Gator_BrakeSimple::Gator_BrakeSimple(const std::string& name) : ChBrakeSimple(name) {}

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono
