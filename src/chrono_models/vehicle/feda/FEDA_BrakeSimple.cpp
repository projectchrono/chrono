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
// FEDA simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/feda/FEDA_BrakeSimple.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double FEDA_BrakeSimple::m_maxtorque = 8000;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FEDA_BrakeSimple::FEDA_BrakeSimple(const std::string& name) : ChBrakeSimple(name) {}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono
